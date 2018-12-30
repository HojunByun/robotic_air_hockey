#include <stdlib.h>
#include <math.h>

#include "serial_inputs.h"
#include "arm_data_struct.h"

#define LENGTH (double)5  // inches
#define SQ(x) (double)(x*x)
#define DEG_TO_RAD(x) ((double)x*PI/180)
#define STEP_ROT DEG_TO_RAD(3)  // 3 degrees

// arm0(left) and arm1(right) setup
store_arm_data arm0;
store_arm_data arm1;

const double start_arm0_theta0 = 2*LENGTH*cos(PI/4);
const double start_arm0_theta1 = 2*LENGTH*sin(PI/4);
const double start_arm1_theta0 = 2*LENGTH*cos(3*PI/4);
const double start_arm1_theta1 = 2*LENGTH*sin(3*PI/4);

void init_pins()
{
  arm0.motor0_step_pin = 2;
  arm0.motor1_step_pin = 3;
  arm0.motor0_dir_pin = 4;
  arm0.motor1_dir_pin = 5;

  arm1.motor0_step_pin = 6;
  arm1.motor1_step_pin = 7;
  arm1.motor0_dir_pin = 8;
  arm1.motor1_dir_pin = 9;
}

void calibrate_motors()
{
  pregame_arm_data *arm_data = read_arm_pregame();
  arm0.arm_theta0 = arm_data->arm0_theta0;
  arm0.arm_theta1 = arm_data->arm0_theta1;
  arm1.arm_theta0 = arm_data->arm1_theta0;
  arm1.arm_theta1 = arm_data->arm1_theta1;
  // free(pregame_arm_data);

  // probably run these two in parallel
  move_arm(arm0, start_arm0_theta0, start_arm0_theta1);
  move_arm(arm1, start_arm1_theta0, start_arm1_theta1);
}

void move_arm(store_arm_data arm, float goal_theta0, float goal_theta1)
{
  float diff0 = goal_theta0 - arm.arm_theta0;
  float diff1 = goal_theta1 - arm.arm_theta1;

  // replace all of this with a ros trajectory
  while(diff0 > 1 && diff1 > 1) {
    // to make stepper motors move one step, generate square wave signal
    digitalWrite(arm.motor0_step_pin, HIGH);
    digitalWrite(arm.motor1_step_pin, HIGH);
    digitalWrite(arm.motor0_step_pin, HIGH);
    digitalWrite(arm.motor1_step_pin, HIGH);
    delayMicroseconds(300);

    digitalWrite(arm.motor0_step_pin, LOW);
    digitalWrite(arm.motor1_step_pin, LOW);
    digitalWrite(arm.motor0_step_pin, LOW);
    digitalWrite(arm.motor1_step_pin, LOW);
    delayMicroseconds(300);
    arm.arm_theta0 += STEP_ROT;  // each step is a certain angle
    arm.arm_theta1 += STEP_ROT;

    // NOTE: need to convert normal stepper rotation to belt translation
    // for second arm since motor drives a belt and armindirectly
    diff0 = goal_theta0 - arm.arm_theta0;
    diff1 = goal_theta1 - arm.arm_theta1;
  }
}


void setup() {
  init_pins();
  // set motor control pin modes
  pinMode(arm0.motor0_step_pin, OUTPUT);
  pinMode(arm0.motor1_step_pin, OUTPUT);
  pinMode(arm0.motor0_dir_pin, OUTPUT);
  pinMode(arm0.motor1_dir_pin, OUTPUT);

  pinMode(arm1.motor0_step_pin, OUTPUT);
  pinMode(arm1.motor1_step_pin, OUTPUT);
  pinMode(arm1.motor0_dir_pin, OUTPUT);
  pinMode(arm1.motor1_dir_pin, OUTPUT);
  // digitalWrite(dirPin,HIGH); //Enables the motor to move in a particular direction
}


void loop() {
//  rec_arm_data *new_goal = read_arm_goals();
//  move_arm(arm0, new_goal->arm_x, new_goal->arm_y);
//  // move_arm(arm1, start_x1, start_y1);
//  free(new_goal);
//
//  // need some complicated function to map desired x, y velocities
//  // to specific rotation speed of the two motors, probably depends on the
//  // current angle of both arms}
//// Function for reading the Potentiometer
//int speedUp() {
//  int customDelay = analogRead(A0); // Reads the potentiometer
//  int newCustom = map(customDelay, 0, 1023, 300,4000); // Convrests the read values of the potentiometer from 0 to 1023 into desireded delay values (300 to 4000)
//  return newCustom;
}
