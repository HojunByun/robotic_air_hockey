#define LENGTH (double)5  // inches
#define SQ(x) (double)(x*x)
#define DEG_TO_RAD(x) ((double)x*PI/180)
#define STEP_ROT DEG_TO_RAD(3)  // 3 degrees
#define CW 1
#define CCW 0

bool
DIRECTION1 = CW,
DIRECTION2 = CCW;

// arm0(left) and arm1(right) setup
store_arm_data arm0;
store_arm_data arm1;

const double start_arm0_theta0 = 2*LENGTH*cos(PI/4);
const double start_arm0_theta1 = 2*LENGTH*sin(PI/4);
const double start_arm1_theta0 = 2*LENGTH*cos(3*PI/4);
const double start_arm1_theta1 = 2*LENGTH*sin(3*PI/4);


void process(String string) {

  // ----- convert string to upper case
  INPUT_STRING = string;
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
}

/* Want a "move_to" function that takes in which arm and desired x and y
   to have both motors moving concurrently, have x and y change simultaenously
   by finding which one is the max and just doing a for-loop through that
   and constantly modify x and y and dec dx and dy until both 0
*/

/* Need to perform basic inverse kinematic calculation: given goal x and y,
   what angle is necessary for motors 1 and 2, then from here calculate
   number of absolute(not relative to current position) steps required
   to move to that from 12 o'clock
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

/* Once we have the absolute number of steps necessary from 12 o'clock,
   need to know number of steps to move relative to current position
   just find absolute value diff and calculate direction if neg or pos
*/


/* Need to calculate delays between movements... if we want both motors
   to reach their goals at the same time, we need to define max and min
   delays bewteen steps(max and min speeds sorta) and then whichever
   motor has to move more steps, have that one have the minimum delay
   and have the other motor just move slower: time1 / its steps
*/

// use millis() or micros() - previous move's time to determine
// time elapsed since last move, and if > this motor's target delay,
// then move motor once and decrement steps needed
