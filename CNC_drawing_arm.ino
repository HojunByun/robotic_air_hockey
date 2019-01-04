/**********************************************************************
  DUAL ARM PLOTTER WITH BELT DRIVE
  Code by lingib
  Last update 5 August 2017

  ----------
  CONCEPT
  ----------
  This plotter is essentially a 2DOF (two degrees of freedom) robotic arm. Unlike
  traditonal arms, which have "shoulder" and "elbow" motors, this plotter has two
  shoulder motors. The second motor controls the "elbow" position by means of a
  lightweight mechanical linkage.

  Software routines mask the natual tendency of the plotter to produce curved lines.

  Simple belt drive gearing improves the plotter resolution by a factor of 4 and 
  allows both shoulder pulleys to share a common shaft.
  
  Advantages;
  1: Eliminating the heavy elbow motor improves the response time. All we are moving
  are lightweight linkages.

  2: Construction is simplified ... all you need are a hacksaw and wood-working drills.

  3: Extremely low cost as very few parts.

  ----------
  COPYRIGHT
  ----------
  This code is free software: you can redistribute it and/or
  modify it under the terms of the GNU General Public License as published
  by the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License. If
  not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

// -------------------------------
// GLOBALS
// -------------------------------

// ----- constants
#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

// ----- motor definitions
#define STEPS_PER_DEG 12800/360     //motor=200 steps/rev; 16 x microstepping; 4 x belt drive
#define NUDGE STEPS_PER_DEG/4       //rotate the motor 0.25 degrees (change number to suit)
#define CW 1                        //motor directions
#define CCW 0
#define DIR1 8                      //Arduino ports
#define DIR2 10
#define STEP1 9
#define STEP2 11

long
PULSE_WIDTH = 2,                    //easydriver step pulse-width (uS)
DELAY_MIN = 2500,                   //minimum inter-step delay (uS) between motor steps (controls speed)
DELAY1,                             //inter-step delay for motor1 (uS)
DELAY2,                             //inter-step delay for motor2 (uS)
STEPS1,                             //motor1 steps from 12 o'clock to reach an XY co-ordinate
STEPS2;                             //motor2 steps from 12 o'clock to reach an XY co-ordinate

// ----- plotter definitions
#define BAUD 9600
#define XOFF 0x13                   //pause transmission (19 decimal)
#define XON 0x11                    //resume transmission (17 decimal)
#define PEN 3

float
OFFSET = 210,                       //motor offset along x_axis
YAXIS = 465,                        //motor heights above (0,0)
LENGTH = 300,                       //length of each arm-segment
SCALE_FACTOR = 1,                   //drawing scale (1 = 100%)
ARC_MAX = 2;                        //maximum arc-length (controls smoothness)

int
/*
   XY plotters only deal in integer steps.
*/
THIS_X = 0,                         //this X co-ordinate (rounded)
THIS_Y = 0,                         //this Y co-ordinate (rounded)
LAST_X = 0,                         //last X co-ordinate (rounded)
LAST_Y = 0;                         //last Y co-ordinate (rounded)

// ----- gcode definitions
#define STRING_SIZE 128             //string size

char
BUFFER[STRING_SIZE + 1],
       INPUT_CHAR;

String
INPUT_STRING,
SUB_STRING;

int
INDEX = 0,                        //buffer index
START,                            //used for sub_string extraction
FINISH;

float
X,                                //gcode float values held here
Y,
I,
J;

// -----------------------
// SETUP
// -----------------------
void setup()
{
  // ----- initialise motor1
  pinMode(DIR1, OUTPUT);
  pinMode(STEP1, OUTPUT);
  digitalWrite(DIR1, CW);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(STEP1, HIGH);

  // ----- initialise motor2
  pinMode(DIR2, OUTPUT);
  pinMode(STEP2, OUTPUT);
  digitalWrite(DIR2, CW);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(STEP2, HIGH);

  // ----- initialise STEPS1, STEPS2 for co-ordinate (0,0)
  calculate_steps(0, 0);

  // ----- pen-lift
  pinMode(PEN, OUTPUT);                                       //D3
  TCCR2A = _BV(COM2B1) | _BV(COM2B0) | _BV(WGM20);            //PWM
  TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS21) | _BV(CS20);    //div 1024
  OCR2A = 156;                                                //20mS period
  OCR2B = 148;                                                //2mS period (pen-up)

  /*
    The above pen-lift comprises a standard servo which requires a 1 mS pulse
    for pen down, or a 2mS pulse for pen up, with a fixed period of 20mS.

    The Arduino "bit value" macro, #define _BV(x) (1 << x), is used to
    set the Timer2 mode to "phase-correct PWM" with a variable "top-limit".
    In this mode the timer counts up to the value entered into register OCR2A
    then back down to zero.

    The following values were used to obtain a 20mS period at pin D3:
      clock:        16 MHz
    prescaler:      1024
    top-limit (OCR2A):  156
    period:       16MHz/1024/(156*2) = 50Hz (20mS)

    If you enter a value into register OCR2B that is LESS than the value in
    register OCR2A then timer2 will will pass through the value in OCR2B
    twice ... once on the way up ... and once on the way down. The duration
    of the output pulse on pin D3 is the time that the count in OCR2A is
    greater than the value in OCR2B.

    A value of 148 entered into OCR2B creates a 1mS pulse:
    period:       156-148)*20mS/156 = 1mS (pen-up)

    A value of 140 entered into OCR2B creates a 2mS pulse):
    period:     156-140)*20mS/156 = 2mS (pen-down)
  */

  // ----- plotter setup
  memset(BUFFER, '\0', sizeof(BUFFER));     //fill with string terminators
  INPUT_STRING.reserve(STRING_SIZE);
  INPUT_STRING = "";

  // ----- establish serial link
  Serial.begin(BAUD);

  // ----- flush the buffers
  Serial.flush();                           //clear TX buffer
  while (Serial.available()) Serial.read(); //clear RX buffer

  // ----- display commands
  menu();
}

//--------------------------------------------------------------------------
// MAIN LOOP
//--------------------------------------------------------------------------
void loop() {

  // ----- get the next instruction
  while (Serial.available()) {
    INPUT_CHAR = (char)Serial.read();         //read character
    Serial.write(INPUT_CHAR);                 //echo character to the screen
    BUFFER[INDEX++] = INPUT_CHAR;             //add char to buffer
    if (INPUT_CHAR == '\n') {                 //check for line feed
      Serial.flush();                         //clear TX buffer
      Serial.write(XOFF);                     //pause transmission
      INPUT_STRING = BUFFER;                  //convert to string
      process();                              //interpret string and perform task
      memset(BUFFER, '\0', sizeof(BUFFER));   //fill buffer with string terminators
      INDEX = 0;                              //point to buffer start
      INPUT_STRING = "";                      //empty string
      Serial.flush();                         //clear TX buffer
      Serial.write(XON);                      //resume transmission
    }
  }
}

//---------------------------------------------------------------------------
// MENU
//---------------------------------------------------------------------------
/*
   The Arduino F() flash macro is used to conserve RAM.
*/
void menu() {
  Serial.println(F(""));
  Serial.println(F("  ------------------------------------------------------"));
  Serial.println(F("                         MENU"));
  Serial.println(F("  ------------------------------------------------------"));
  Serial.println(F("    MENU ............... menu"));
  Serial.println(F("    G00 X## Y## ........ goto XY (pen-up)"));
  Serial.println(F("    G01 X## Y## ........ goto XY (pen-down)"));
  Serial.println(F("    T1 ................. move pen to 0,0"));
  Serial.println(F("    T2 S##.## .......... set drawing Scale (1=100%)"));
  Serial.println(F("    T3 ................. pen up"));
  Serial.println(F("    T4 ................. pen down"));
  Serial.println(F("    T5 ................. test pattern: ABC"));
  Serial.println(F("    T6 ................. test pattern: target"));
  Serial.println(F("    T7 ................. test pattern: radials"));
  Serial.println(F("  ------------------------------------------------------"));
}

//--------------------------------------------------------------------------
// PROCESS
//--------------------------------------------------------------------------
void process() {

  // ----- convert string to upper case
  INPUT_STRING.toUpperCase();

  // ----------------------------------
  // G00   linear move with pen_up
  // ----------------------------------
  if (INPUT_STRING.startsWith("G00")) {

    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      X = SUB_STRING.toFloat();
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      Y = SUB_STRING.toFloat();
    }

    pen_up();
    move_to(X, Y);
  }

  // ----------------------------------
  // G01   linear move with pen_down
  // ----------------------------------
  if (INPUT_STRING.startsWith("G01")) {

    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      X = SUB_STRING.toFloat();
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      Y = SUB_STRING.toFloat();
    }

    pen_down();
    move_to(X, Y);
  }

  // ----------------------------------
  // G02   clockwise arc with pen_down
  // ----------------------------------
  if (INPUT_STRING.startsWith("G02")) {

    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('X'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      X = SUB_STRING.toFloat();
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('Y'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      Y = SUB_STRING.toFloat();
    }

    // ----- extract I
    START = INPUT_STRING.indexOf('I');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('I'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      I = SUB_STRING.toFloat();
    }

    // ----- extract J
    START = INPUT_STRING.indexOf('J');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('J'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      J = SUB_STRING.toFloat();
    }

    pen_down();
    draw_arc_cw(X, Y, I, J);
  }

  // ------------------------------------------
  // G03   counter-clockwise arc with pen_down
  // ------------------------------------------
  if (INPUT_STRING.startsWith("G03")) {

    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('X'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      X = SUB_STRING.toFloat();
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('Y'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      Y = SUB_STRING.toFloat();
    }

    // ----- extract I
    START = INPUT_STRING.indexOf('I');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('I'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      I = SUB_STRING.toFloat();
    }

    // ----- extract J
    START = INPUT_STRING.indexOf('J');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('J'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      J = SUB_STRING.toFloat();
    }

    pen_down();
    draw_arc_ccw(X, Y, I, J);
  }

  // ----------------------------------
  // MENU
  // ----------------------------------
  if (INPUT_STRING.startsWith("MENU")) {
    menu();
  }

  // ----------------------------------
  // T1   position the pen over 0,0
  // ----------------------------------
  if (INPUT_STRING.startsWith("T1")) {

    // ----- variables
    int step;           //loop counter
    int steps = NUDGE;  //steps motor is to rotate

    // ----- instructions
    Serial.println(F(""));
    Serial.println(F("  ----------------------------------------------"));
    Serial.println(F("    Position the pen over the 0,0 co-ordinate:"));
    Serial.println(F("  ----------------------------------------------"));
    Serial.println(F("    X-axis:             Y-axis:"));
    Serial.println(F("   'A'  'S'            'K'  'L'"));
    Serial.println(F("   <-    ->            <-    ->"));
    Serial.println(F("    Exit = 'E'"));

    // ----- flush the buffer
    while (Serial.available() > 0) Serial.read();

    // ----- control motors with 'A', 'S', 'K', and 'L' keys

    char keystroke = ' ';
    while (keystroke != 'E') {  //press 'E' key to exit

      // ----- check for keypress
      if (Serial.available() > 0) {
        keystroke = (char) Serial.read();
      }

      // ----- select task
      switch (keystroke) {
        case 'a':
        case 'A': {
            // ----- rotate motor1 CW
            for (step = 0; step < steps; step++) {
              step1_cw();
            }
            keystroke = ' ';    //otherwise motor will continue to rotate
            break;
          }
        case 's':
        case 'S': {
            // ------ rotate motor1 CCW
            for (step = 0; step < steps; step++) {
              step1_ccw();
            }
            keystroke = ' ';
            break;
          }
        case 'k':
        case 'K': {
            // ----- rotate motor2 CW
            for (step = 0; step < steps; step++) {
              step2_cw();
            }
            keystroke = ' ';
            break;
          }
        case 'l':
        case 'L': {
            // ----- rotate motor2 CCW
            for (step = 0; step < steps; step++) {
              step2_ccw();
            }
            keystroke = ' ';
            break;
          }
        case 'e':
        case 'E': {
            // ----- exit
            Serial.println(F(" "));
            Serial.println(F("  Calibration complete ..."));
            keystroke = 'E';
            break;
          }
        // ----- default for keystroke
        default: {
            break;
          }
      }
    }

    // ----- initialise counters for co-ordinate (0,0)
    calculate_steps(0, 0);        //initialise STEPS1, STEPS2
    THIS_X = 0;                   //current X co-ordinate
    THIS_Y = 0;                   //current Y co-ordinate
    LAST_X = 0;                   //previous X co-ordinate
    LAST_Y = 0;                   //previous Y-co-ordinate
  }

  // ----------------------------------
  // T2   set scale factor
  // ----------------------------------
  if (INPUT_STRING.startsWith("T2")) {
    Serial.println("T2");

    START = INPUT_STRING.indexOf('S');
    if (!(START < 0)) {
      FINISH = START + 6;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH);
      SCALE_FACTOR = SUB_STRING.toFloat();
      Serial.print(F("Drawing now ")); Serial.print(SCALE_FACTOR * 100); Serial.println(F("%"));
    }
    else {
      Serial.println(F("Invalid scale factor ... try again. (1 = 100%)"));
    }
  }

  // ----------------------------------
  // T3   pen up
  // ----------------------------------
  if (INPUT_STRING.startsWith("T3")) {
    pen_up();
  }

  // ----------------------------------
  // T4   pen down
  // ----------------------------------
  if (INPUT_STRING.startsWith("T4")) {
    pen_down();
  }

  // ----------------------------------
  // T5   ABC test pattern
  // ----------------------------------
  if (INPUT_STRING.startsWith("T5")) {
    abc();
  }

  // ----------------------------------
  // T6   target test pattern
  // ----------------------------------
  if (INPUT_STRING.startsWith("T6")) {
    target();
  }

  // ----------------------------------
  // T7   radial line test pattern
  // ----------------------------------
  if (INPUT_STRING.startsWith("T7")) {
    radials();
  }
}

// -------------------------------
// MOVE_TO
// -------------------------------
/*
   We need to make this plotter think that it is an XY plotter
*/
void move_to(float x, float y) {

  // ----- apply scale factor
  THIS_X = (int)((round)(x * SCALE_FACTOR));
  THIS_Y = (int)((round)(y * SCALE_FACTOR));

  // ----- draw a line between these co-ordinates
  draw_line(LAST_X, LAST_Y, THIS_X, THIS_Y);

  // ----- remember last rounded co-ordinate
  LAST_X = THIS_X;
  LAST_Y = THIS_Y;
}

// ------------------------------------------------------------------------
// DRAW LINE
// ------------------------------------------------------------------------
/*
  This routine assumes that we are talking to a linear XY plotter.

  The algorithm automatically maps all "octants" to "octant 0" and
  automatically swaps the XY coordinates if dY is greater than dX. A swap
  flag determines which motor moves for any combination X,Y inputs. The swap
  algorithm is further optimised by realising that dY is always positive
  in quadrants 0,1 and that dX is always positive in "quadrants" 0,3.

  Each intermediate XY co-ordinate is plotted which results in a straight line
*/
void draw_line(int x1, int y1, int x2, int y2) {

  // ----- locals
  int
  x = x1,                             //current X-axis position
  y = y1,                             //current Y-axis position
  dy,                                 //line slope
  dx,
  slope,
  longest,                            //axis lengths
  shortest,
  maximum,
  error,                              //bresenham thresholds
  threshold;

  // ----- find longest and shortest axis
  dy = y2 - y1;                         //vertical distance
  dx = x2 - x1;                         //horizontal distance
  longest = max(abs(dy), abs(dx));      //longest axis
  shortest = min(abs(dy), abs(dx));     //shortest axis

  // ----- scale Bresenham values by 2*longest
  error = -longest;                     //add offset to so we can test at zero
  threshold = 0;                        //test now done at zero
  maximum = (longest << 1);             //multiply by two
  slope = (shortest << 1);              //multiply by two ... slope equals (shortest*2/longest*2)

  // ----- initialise the swap flag
  /*
     The XY axes are automatically swapped by using "longest" in
     the "for loop". XYswap is used to decode the motors.
  */
  bool XYswap = true;                    //used for motor decoding
  if (abs(dx) >= abs(dy)) XYswap = false;

  // ----- pretend we are always in octant 0
  /*
     The current X-axis and Y-axis positions will now be incremented (decremented) each time
     through the loop. These intermediate steps are parsed to the plot(x,y) function which calculates
     the number of steps required to reach each of these intermediate co-ordinates. This effectively
     linearises the plotter and eliminates unwanted curves.
  */
  for (int i = 0; i < longest; i++) {

    // ----- move left/right along X axis
    if (XYswap) {   //swap
      if (dy < 0) {
        y--;
      } else {
        y++;
      }
    } else {    //no swap
      if (dx < 0) {
        x--;
      } else {
        x++;
      }
    }

    // ----- move up/down Y axis
    error += slope;
    if (error > threshold) {
      error -= maximum;

      // ----- move up/down along Y axis
      if (XYswap) {  //swap
        if (dx < 0) {
          x--;
        } else {
          x++;
        }
      } else {  //no swap
        if (dy < 0) {
          y--;
        } else {
          y++;
        }
      }
    }

    // ----- plot the next rounded XY co-ordinate
    plot(x, y);
  }
}

//----------------------------------------------------------------------------------------
// PLOT
//----------------------------------------------------------------------------------------
/*
  This routine effectively linearises the plotter and eliminates unwanted curves by
  stepping motor1 and motor2 to the XY co-ordinate that is nearest the target location.
  The global step-counters STEPS1, STEPS2 are automatically updated.
*/
void plot(int x, int y ) {

  // ----- locals
  float
  x_axis = (float)x,                  //calculate_steps() requires a float
  y_axis = (float)y;

  long
  current_steps1 = STEPS1,            //current motor steps
  current_steps2 = STEPS2,
  steps1,                             //extra motor steps to reach this co-ordinate
  steps2;

  bool
  direction1,                         //motor directions
  direction2;

  long
  current_time,                       //system time
  previous_time1,                     //previous system time for motor1
  previous_time2;                     //previous system time for motor2

  // ----- calculate extra motor steps
  calculate_steps(x_axis, y_axis);              //calculate fresh values for STEPS1, STEPS2
  steps1 = abs(STEPS1 - current_steps1);        //extra steps required
  steps2 = abs(STEPS2 - current_steps2);        //extra steps required

  // ----- calculate the motor directions
  direction1 = (STEPS1 > current_steps1) ? CW : CCW;
  direction2 = (STEPS2 > current_steps2) ? CW : CCW;

  // ----- calculate motor delays for the extra steps
  calculate_delays(steps1, steps2);

  // ----- preload the timers and counters
  previous_time1 = micros();                      //reset the timer
  previous_time2 = micros();                      //reset the timer

  // ----- now step the motors
  /*
     steps1 and steps2 are now used as down-counters
  */
  while ((steps1 != 0) || (steps2 != 0)) {        //stop when both counters equal zero
    // ----- step motor1
    if (steps1 > 0) {                             //prevent additional step ... it occasionally happens!
      current_time = micros();
      if (current_time - previous_time1 > DELAY1) {
        previous_time1 = current_time;              //reset timer
        steps1--;                                   //decrement counter1
        step_motor1(direction1);
      }
    }
    // ----- step motor2
    if (steps2 > 0) {                             //prevent additional step ... it occasionally happens!
      current_time = micros();
      if (current_time - previous_time2 > DELAY2) {
        previous_time2 = current_time;              //reset timer
        steps2--;                                   //decrement counter2
        step_motor2(direction2);
      }
    }
  }
}

//----------------------------------------------------------------------------------------
// CALCULATE STEPS
//----------------------------------------------------------------------------------------
/*
  This routine calculates the the number of motor steps to reach (x,y) from 12 o'clock .
  Global variables STEPS1, STEPS2 hold the required number of steps
*/
void calculate_steps(float x, float y) {

  // ----- locals
  float
  distance,             //pen distance to motors
  angle1,               //motor1 angle
  angle2;               //motor2 angle

  // ----- calculate distances
  distance = sqrt((OFFSET - x) * (OFFSET - x) + (YAXIS - y) * (YAXIS - y));

  // ----- calculate motor1 angle when pen at (x,y)
  if (x > OFFSET) {
    angle1 = PI + acos(distance / (2 * LENGTH)) - atan((x - OFFSET) / (YAXIS - y)); //radians
  } else {
    angle1 = PI + acos(distance / (2 * LENGTH)) + atan((OFFSET - x) / (YAXIS - y)); //radians
  }

  // ----- calculate motor2 angle when pen at (x,y)
  if (x > OFFSET) {
    angle2 = PI - acos(distance / (2 * LENGTH)) - atan((x - OFFSET) / (YAXIS - y)); //radians
  } else {
    angle2 = PI - acos(distance / (2 * LENGTH)) + atan((OFFSET - x) / (YAXIS - y)); //radians
  }

  // ----- calculate steps required to reach (x,y) from 12 o'clock
  STEPS1 = int(angle1 * RAD_TO_DEG * STEPS_PER_DEG);
  STEPS2 = int(angle2 * RAD_TO_DEG * STEPS_PER_DEG);
}

//---------------------------------------------------------------------------
// CALCULATE DELAYS
//---------------------------------------------------------------------------
/*
   Assigns values to DELAY1, DELAY2 ready for next pen move
*/
void calculate_delays(long steps1, long steps2) {

  // ----- locals
  float
  rotate_time;

  long
  min_steps,
  max_steps,
  delay_max;

  // ----- find max and min number of steps
  max_steps = max(steps1, steps2);

  min_steps = min(steps1, steps2);

  // ----- calculate the total time for to complete the move
  rotate_time = (float)(max_steps * DELAY_MIN);

  // ----- calculate delay for motor with min_steps
  if (min_steps < 1) min_steps = 1;   //prevent possible divide by zero
  delay_max = (long)(rotate_time / ((float)min_steps));

  // ----- assign delays to each motor
  DELAY1 = (steps1 > steps2) ? DELAY_MIN : delay_max;
  DELAY2 = (steps1 > steps2) ? delay_max : DELAY_MIN;
}

//----------------------------------------------------------------------------
// DRAW ARC CLOCKWISE (G02)
//----------------------------------------------------------------------------
void draw_arc_cw(float x, float y, float i, float j) {

  // ----- inkscape sometimes produces some crazy values for i,j
  if ((i < -100) || (i > 100) || (j < -100) || (j > 100)) {
    move_to(x, y);
  } else {

    // ----- variables
    float
    thisX = LAST_X / SCALE_FACTOR, //current unscaled X co-ordinate
    thisY = LAST_Y / SCALE_FACTOR, //current unscaled Y co-ordinate
    nextX = x,                    //next X co-ordinate
    nextY = y,                    //next Y co-ordinate
    newX,                         //interpolated X co-ordinate
    newY,                         //interpolated Y co-ordinate
    I = i,                        //horizontal distance thisX from circle center
    J = j,                        //vertical distance thisY from circle center
    circleX = thisX + I,          //circle X co-ordinate
    circleY = thisY + J,          //circle Y co-ordinate
    delta_x,                      //horizontal distance between thisX and nextX
    delta_y,                      //vertical distance between thisY and nextY
    chord,                        //line_length between lastXY and nextXY
    radius,                       //circle radius
    alpha,                        //interior angle of arc
    beta,                         //fraction of alpha
    arc,                          //subtended by alpha
    current_angle,                //measured CCW from 3 o'clock
    next_angle;                   //measured CCW from 3 o'clock

    // ----- calculate arc
    delta_x = thisX - nextX;
    delta_y = thisY - nextY;
    chord = sqrt(delta_x * delta_x + delta_y * delta_y);
    radius = sqrt(I * I + J * J);
    alpha = 2 * asin(chord / (2 * radius)); //see construction lines
    arc = alpha * radius;         //radians

    // ----- sub-divide alpha
    int segments = 1;
    if (arc > ARC_MAX) {
      segments = (int)(arc / ARC_MAX);
      beta = alpha / segments;
    } else {
      beta = alpha;
    }

    // ----- calculate current angle
    /*
      atan2() angles between 0 and PI are CCW +ve from 3 o'clock.
      atan2() angles between 2*PI and PI are CW -ve relative to 3 o'clock
    */
    current_angle = atan2(-J, -I);
    if (current_angle <= 0) current_angle += 2 * PI; //angles now 360..0 degrees CW

    // ----- plot intermediate CW co-ordinates
    next_angle = current_angle;                             //initialise angle
    for (int segment = 1; segment < segments; segment++) {
      next_angle -= beta;                                   //move CW around circle
      if (next_angle < 0) next_angle += 2 * PI;             //check if angle crosses zero
      newX = circleX + radius * cos(next_angle);            //standard circle formula
      newY = circleY + radius * sin(next_angle);
      move_to(newX, newY);
    }

    // ----- draw final line
    move_to(nextX, nextY);
  }
}

//----------------------------------------------------------------------------
// DRAW ARC COUNTER-CLOCKWISE (G03)
//----------------------------------------------------------------------------
/*
   We know the start and finish co-ordinates which allows us to calculate the
   chord length. We can also calculate the radius using the biarc I,J values.
   If we bisect the chord the center angle becomes 2*asin(chord/(2*radius)).
   The arc length may now be calculated using the formula arc_length = radius*angle.
*/
void draw_arc_ccw(float x, float y, float i, float j) {

  // ----- inkscape sometimes produces some crazy values for i,j
  if ((i < -100) || (i > 100) || (j < -100) || (j > 100)) {
    move_to(x, y);
  } else {

    // ----- variables
    float
    thisX = LAST_X / SCALE_FACTOR,  //current unscaled X co-ordinate
    thisY = LAST_Y / SCALE_FACTOR,  //current unscaled Y co-ordinate
    nextX = x,                      //next X co-ordinate
    nextY = y,                      //next Y co-ordinate
    newX,                           //interpolated X co-ordinate
    newY,                           //interpolated Y co-ordinate
    I = i,                          //horizontal distance thisX from circle center
    J = j,                          //vertical distance thisY from circle center
    circleX = thisX + I,            //circle X co-ordinate
    circleY = thisY + J,            //circle Y co-ordinate
    delta_x,                        //horizontal distance between thisX and nextX
    delta_y,                        //vertical distance between thisY and nextY
    chord,                          //line_length between lastXY and nextXY
    radius,                         //circle radius
    alpha,                          //interior angle of arc
    beta,                           //fraction of alpha
    arc,                            //subtended by alpha
    current_angle,                  //measured CCW from 3 o'clock
    next_angle;                     //measured CCW from 3 o'clock

    // ----- calculate arc
    delta_x = thisX - nextX;
    delta_y = thisY - nextY;
    chord = sqrt(delta_x * delta_x + delta_y * delta_y);
    radius = sqrt(I * I + J * J);
    alpha = 2 * asin(chord / (2 * radius));     //see construction lines
    arc = alpha * radius;                       //radians

    // ----- sub-divide alpha
    int segments = 1;
    if (arc > ARC_MAX) {
      segments = (int)(arc / ARC_MAX);
      beta = alpha / segments;
    } else {
      beta = alpha;
    }

    // ----- calculate current angle
    /*
        tan2() angles between 0 and PI are CCW +ve from 3 o'clock.
        atan2() angles between 2*PI and PI are CW -ve relative to 3 o'clock
    */
    current_angle = atan2(-J, -I);
    if (current_angle <= 0) current_angle += 2 * PI;        //angles now 360..0 degrees CW

    // ----- plot intermediate CCW co-ordinates
    next_angle = current_angle;                             //initialise angle
    for (int segment = 1; segment < segments; segment++) {
      next_angle += beta;                                   //move CCW around circle
      if (next_angle > 2 * PI) next_angle -= 2 * PI;        //check if angle crosses zero
      newX = circleX + radius * cos(next_angle);            //standard circle formula
      newY = circleY + radius * sin(next_angle);
      move_to(newX, newY);
    }

    // ----- draw final line
    move_to(nextX, nextY);
  }
}

//--------------------------------------------------------------------
// STEP MOTOR1
//--------- -----------------------------------------------------------
void step_motor1(bool dir1) {
  if (dir1 == CW) {
    step1_cw();
  } else {
    step1_ccw();
  }
}

//--------------------------------------------------------------------
// STEP MOTOR2
//--------- -----------------------------------------------------------
void step_motor2(bool dir2) {
  if (dir2 == CW) {
    step2_cw();
  } else {
    step2_ccw();
  }
}

//--------------------------------------------------------------------
// STEP1_CW        (step actuator motor1 clockwise)
//--------- -----------------------------------------------------------
void step1_cw() {

  //----- set direction
  digitalWrite(DIR1, CW);
  delayMicroseconds(PULSE_WIDTH);

  //----- step motor
  digitalWrite(STEP1, HIGH);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(STEP1, LOW);
  delayMicroseconds(PULSE_WIDTH);
}

//--------------------------------------------------------------------
// STEP1_CCW        (step actuator motor1 counter-clockwise)
//--------- -----------------------------------------------------------
void step1_ccw() {

  //----- set direction
  digitalWrite(DIR1, CCW);
  delayMicroseconds(PULSE_WIDTH);

  //----- step motor
  digitalWrite(STEP1, HIGH);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(STEP1, LOW);
  delayMicroseconds(PULSE_WIDTH);
}

//--------------------------------------------------------------------
// STEP2_CW        (step actuator motor2 clockwise)
//--------- -----------------------------------------------------------
void step2_cw() {

  //----- set direction
  digitalWrite(DIR2, CW);
  delayMicroseconds(PULSE_WIDTH);

  //----- step motor
  digitalWrite(STEP2, HIGH);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(STEP2, LOW);
  delayMicroseconds(PULSE_WIDTH);
}

//--------------------------------------------------------------------
// STEP2_CCW        (step actuator motor2 counter-clockwise)
//--------- -----------------------------------------------------------
void step2_ccw() {

  //----- set direction
  digitalWrite(DIR2, CCW);
  delayMicroseconds(PULSE_WIDTH);

  //----- step motor
  digitalWrite(STEP2, HIGH);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(STEP2, LOW);
  delayMicroseconds(PULSE_WIDTH);
}

//---------------------------------------------------------------------------
// PEN_UP
// Raise the pen
// Changing the value in OCR2B changes the pulse-width to the SG-90 servo
//---------------------------------------------------------------------------
void pen_up() {
  OCR2B = 148;                //1mS pulse
  delay(250);                 //give pen-lift time to respond
}

//---------------------------------------------------------------------------
// PEN_DOWN
// Lower the pen
// Changing the value in OCR2B changes the pulse-width to the SG-90 servo
//---------------------------------------------------------------------------
void pen_down() {
  OCR2B = 140;                //2mS pulse
  delay(250);                 //give pen-lift time to respond
}


// ----------------------------------------
// ABC
// ----------------------------------------
void abc() {

  // ----- letter C
  pen_up();
  move_to( 46.077581, 8.038555 );
  pen_down();
  move_to( 44.484913, 8.164859 );
  move_to( 43.092675, 8.514975 );
  move_to( 41.810333, 9.119737 );
  move_to( 40.749474, 9.934507 );
  move_to( 39.898294, 10.964664 );
  move_to( 39.222991, 12.287431 );
  move_to( 38.836654, 13.724444 );
  move_to( 38.688233, 15.544577 );
  move_to( 38.827317, 17.240735 );
  move_to( 39.203543, 18.665602 );
  move_to( 39.848521, 19.983646 );
  move_to( 40.700859, 21.067141 );
  move_to( 41.728238, 21.900614 );
  move_to( 43.034337, 22.554734 );
  move_to( 44.459619, 22.940826 );
  move_to( 46.087304, 23.079767 );
  move_to( 46.999087, 23.050125 );
  move_to( 47.730461, 22.972816 );
  move_to( 48.465078, 22.853756 );
  move_to( 49.091658, 22.710298 );
  move_to( 49.735578, 22.504820 );
  move_to( 50.268119, 22.292217 );
  move_to( 50.799881, 22.061981 );
  move_to( 51.201509, 21.874136 );
  move_to( 51.201509, 18.364194 );
  move_to( 50.773706, 18.364194 );
  move_to( 50.497832, 18.593065 );
  move_to( 50.083384, 18.918397 );
  move_to( 49.664637, 19.224080 );
  move_to( 49.149994, 19.550380 );
  move_to( 48.602240, 19.833093 );
  move_to( 47.992979, 20.075412 );
  move_to( 47.361382, 20.233433 );
  move_to( 46.660952, 20.289314 );
  move_to( 45.884644, 20.224242 );
  move_to( 45.163636, 20.036522 );
  move_to( 44.498101, 19.720535 );
  move_to( 43.851054, 19.219805 );
  move_to( 43.355966, 18.613137 );
  move_to( 42.917663, 17.751657 );
  move_to( 42.668402, 16.819076 );
  move_to( 42.567640, 15.534855 );
  move_to( 42.678469, 14.195736 );
  move_to( 42.946831, 13.259712 );
  move_to( 43.413758, 12.401478 );
  move_to( 43.919115, 11.820734 );
  move_to( 45.241419, 11.052632 );
  move_to( 45.978754, 10.883200 );
  move_to( 46.680397, 10.829005 );
  move_to( 47.353314, 10.879912 );
  move_to( 48.022146, 11.033184 );
  move_to( 48.674696, 11.277806 );
  move_to( 49.256945, 11.587387 );
  move_to( 49.723404, 11.883902 );
  move_to( 50.141723, 12.190202 );
  move_to( 50.546004, 12.515932 );
  move_to( 50.812596, 12.744402 );
  move_to( 51.201509, 12.744402 );
  move_to( 51.201509, 9.283076 );
  move_to( 50.656313, 9.041645 );
  move_to( 50.161168, 8.826105 );
  move_to( 49.659311, 8.628995 );
  move_to( 49.120826, 8.456637 );
  move_to( 48.406883, 8.270693 );
  move_to( 47.788799, 8.145506 );
  move_to( 47.163646, 8.072462 );
  move_to( 46.077581, 8.038555 );
  move_to( 46.077581, 8.038555 );
  pen_up();

  // ----- letter B
  move_to( 36.753391, 12.754125 );
  pen_down();
  move_to( 36.638363, 11.732067 );
  move_to( 36.325587, 10.877621 );
  move_to( 35.824865, 10.121077 );
  move_to( 35.168569, 9.506701 );
  move_to( 34.268011, 8.957841 );
  move_to( 33.282343, 8.592755 );
  move_to( 32.250726, 8.402320 );
  move_to( 30.686347, 8.320518 );
  move_to( 24.366511, 8.320518 );
  move_to( 24.366511, 22.797804 );
  move_to( 29.986303, 22.797804 );
  move_to( 31.733546, 22.757964 );
  move_to( 32.543406, 22.681131 );
  move_to( 33.339527, 22.498523 );
  move_to( 34.157395, 22.165821 );
  move_to( 34.894695, 21.654645 );
  move_to( 35.382471, 21.037973 );
  move_to( 35.681264, 20.308309 );
  move_to( 35.790830, 19.433707 );
  move_to( 35.644134, 18.429163 );
  move_to( 35.236630, 17.576648 );
  move_to( 34.591894, 16.889091 );
  move_to( 33.671256, 16.332124 );
  move_to( 33.671256, 16.254343 );
  move_to( 34.990077, 15.791394 );
  move_to( 35.917229, 15.087603 );
  move_to( 36.517941, 14.109174 );
  move_to( 36.753391, 12.754125 );
  move_to( 36.753391, 12.754125 );
  pen_up();

  move_to( 32.883707, 12.812463 );
  pen_down();
  move_to( 32.799937, 13.475254 );
  move_to( 32.611467, 13.872250 );
  move_to( 32.293975, 14.174883 );
  move_to( 31.707244, 14.436175 );
  move_to( 31.264089, 14.524544 );
  move_to( 30.501613, 14.572293 );
  move_to( 29.733495, 14.579493 );
  move_to( 28.897349, 14.582030 );
  move_to( 28.080630, 14.582030 );
  move_to( 28.080630, 10.974863 );
  move_to( 28.352869, 10.974863 );
  move_to( 29.927944, 10.978253 );
  move_to( 30.608564, 10.984572 );
  move_to( 31.276168, 11.055991 );
  move_to( 31.862810, 11.237364 );
  move_to( 32.385104, 11.547787 );
  move_to( 32.660082, 11.879072 );
  move_to( 32.883707, 12.812463 );
  move_to( 32.883707, 12.812463 );
  pen_up();

  move_to( 31.969761, 18.704495 );
  pen_down();
  move_to( 31.925661, 19.053577 );
  move_to( 31.785027, 19.423984 );
  move_to( 31.553429, 19.721988 );
  move_to( 31.153044, 19.958739 );
  move_to( 30.736285, 20.070609 );
  move_to( 30.132147, 20.124028 );
  move_to( 29.529231, 20.137196 );
  move_to( 28.430652, 20.143473 );
  move_to( 28.080630, 20.143473 );
  move_to( 28.080630, 17.080783 );
  move_to( 28.663999, 17.080783 );
  move_to( 29.548576, 17.089343 );
  move_to( 30.171037, 17.109951 );
  move_to( 30.782282, 17.184377 );
  move_to( 31.153044, 17.304408 );
  move_to( 31.599191, 17.587578 );
  move_to( 31.814194, 17.878055 );
  move_to( 31.927043, 18.246639 );
  move_to( 31.969761, 18.704495 );
  move_to( 31.969761, 18.704495 );
  pen_up();

  // ----- letter A
  move_to( 22.266381, 8.320518 );
  pen_down();
  move_to( 18.406419, 8.320518 );
  move_to( 17.404968, 11.247086 );
  move_to( 12.037969, 11.247086 );
  move_to( 11.036518, 8.320518 );
  move_to( 7.273784, 8.320518 );
  move_to( 12.621338, 22.797804 );
  move_to( 16.918827, 22.797804 );
  move_to( 22.266381, 8.320518 );
  pen_up();

  move_to( 16.500745, 13.901420 );
  pen_down();
  move_to( 14.721468, 19.093409 );
  move_to( 12.942191, 13.901420 );
  move_to( 16.500745, 13.901420 );
  pen_up();

  // ----- home
  move_to( 0.0000, 0.0000 );
}

//----------------------------------------------------------------------------
// TARGET test pattern
//----------------------------------------------------------------------------
void target() {

  // ----- circle
  pen_up();
  move_to(130, 100);
  pen_down();
  move_to(128, 88);
  move_to(122, 79);
  move_to(112, 73);
  move_to(100, 70);
  move_to(87, 73);
  move_to(78, 79);
  move_to(71, 88);
  move_to(69, 100);
  move_to(71, 111);
  move_to(78, 123);
  move_to(87, 130);
  move_to(100, 132);
  move_to(112, 130);
  move_to(122, 123);
  move_to(129, 110);
  move_to(130, 100);
  pen_up();

  // ----- back-slash
  pen_up();
  move_to(50, 150);
  pen_down();
  move_to(78, 123);
  move_to(100, 100);
  move_to(123, 79);
  move_to(150, 50);
  pen_up();

  // ----- slash
  pen_up();
  move_to(50, 50);
  pen_down();
  move_to(78, 79);
  move_to(100, 100);
  move_to(122, 123);
  move_to(150, 150);
  pen_up();

  // ----- square
  pen_up();
  move_to(50, 150);
  pen_down();
  move_to(100, 150);
  move_to(150, 150);
  move_to(150, 100);
  move_to(150, 50);
  move_to(100, 50);
  move_to(50, 50);
  move_to(50, 100);
  move_to(50, 150);
  pen_up();

  // ------ home
  move_to(0.0000, 0.0000);
}

//----------------------------------------------------------------------------
// RADIALS test pattern
//----------------------------------------------------------------------------
void radials() {

  // ----- move to the centre of the square
  pen_up();
  move_to(100, 100);

  // ----- draw octant 0 radials
  pen_down();
  move_to(150, 100);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(150, 125);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(150, 150);
  pen_up();
  move_to(100, 100);

  // ----- draw octant 1 radials
  pen_down();
  move_to(125, 150);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(100, 150);
  pen_up();
  move_to(100, 100);

  // ----- draw octant 2 radials
  pen_down();
  move_to(75, 150);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(50, 150);
  pen_up();
  move_to(100, 100);

  // ----- draw octant 3 radials
  pen_down();
  move_to(50, 125);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(50, 100);
  pen_up();
  move_to(100, 100);

  // ----- draw octant 4 radials
  pen_down();
  move_to(50, 75);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(50, 50);
  pen_up();
  move_to(100, 100);

  // ----- draw octant 5 radials
  pen_down();
  move_to(75, 50);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(100, 50);
  pen_up();
  move_to(100, 100);

  // ----- draw octant 6 radials
  pen_down();
  move_to(125, 50);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(150, 50);
  pen_up();
  move_to(100, 100);

  // ----- draw octant 7 radials
  pen_down();
  move_to(150, 75);
  pen_up();
  move_to(100, 100);
  pen_up();

  // ----- draw box
  move_to(50, 50);
  pen_down();
  move_to(50, 150);
  move_to(150, 150);
  move_to(150, 50);
  move_to(50, 50);
  pen_up();

  // home --------------
  move_to(0.0000, 0.0000);
}

