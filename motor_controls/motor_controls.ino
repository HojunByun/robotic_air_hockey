#include <stdlib.h>
#include <math.h>

#include "serial_inputs.h"
#include "arm_data_struct.h"
#include "AccelStepper.h"
#include "MultiStepper.h"

#define LENGTH (double)5  // inches
#define SQ(x) (double)(x*x)
#define DEG_TO_RAD(x) ((double)x*PI/180)
#define STEP_ROT DEG_TO_RAD(3)  // 3 degrees
// TODO: Need to check number of steps for full rotation
// one example uses 1350-ish, but other uses 200
#define DEG_TO_STEP(x) ((x == 0) 0 : ((float)200/360)*(float)x)

// define motor pins
#define ARM0_M0_STEP 2
#define ARM0_M0_DIR 3
#define ARM0_M0_STEP 2
#define ARM0_M0_DIR 3
#define ARM0_M0_STEP 2
#define ARM0_M0_DIR 3
#define ARM0_M0_STEP 2
#define ARM0_M0_DIR 3

// arm0(left) and arm1(right) setup
AccelStepper arm0_m0(AccelStepper::DRIVER, ARM0_M0_STEP, ARM0_M0_DIR);
AccelStepper arm0_m0(AccelStepper::DRIVER, ARM0_M0_STEP, ARM0_M0_DIR);
AccelStepper arm0_m0(AccelStepper::DRIVER, ARM0_M0_STEP, ARM0_M0_DIR);
AccelStepper arm0_m0(AccelStepper::DRIVER, ARM0_M0_STEP, ARM0_M0_DIR);
AccelStepper arm0_m0(AccelStepper::DRIVER, ARM0_M0_STEP, ARM0_M0_DIR);

// all stepper motors controlled by one interface
MultiStepper all_motors;
long motor_positions[4] = {0.0, 0.0, 0.0, 0.0};

void calibrate_motors()
{
  pregame_arm_data *arm_data = read_arm_pregame();
  // remember to take note that the arms for some reason may not be visible
  // whatsoever, so also have a boolean in the struct that says whether
  // the arm angles were all sucessfully detected, and if not have the
  // arduino give some warning to users that calib unsuccessful
  arm0_m0.setCurrentPosition(DEG_TO_STEP(arm_data->arm0_theta0));
  motor_positions[0] = DEG_TO_STEP(arm_data->arm0_theta0);
  ..........
  arm0.arm_theta1 = arm_data->arm0_theta1;
  arm1.arm_theta0 = arm_data->arm1_theta0;
  arm1.arm_theta1 = arm_data->arm1_theta1;
  free(pregame_arm_data);
}

void calculate_steps(float x, float y) {

  // ----- locals
  float
  ,             //pen distance to motors
  angle0,               //motor1 angle
  angle1;               //motor2 angle

  // ----- calculate distances
  // distance = sqrt((OFFSET - x) * (OFFSET - x) + (YAXIS - y) * (YAXIS - y));

  // with 2 DOF, there are usually two different solutions to a position
  // temporarily just default to one
  // assum both arm links are same length, so 2*SQ(LENGTH)
  angle1 = acos((SQ(x) + SQ(y) - 2*SQ(LENGTH))/2*SQ(LENGTH));
  angle0 = atan(y/x) - atan((LENGTH*sin(angle1))/(LENGTH + LENGTH*cos(angle1)));

  // ----- calculate steps required to reach (x,y) from 12 o'clock
  STEPS0 = int(DEG_TO_STEP(RAD_TO_DEG(angle0)));
  STEPS1 = int(DEG_TO_STEP(RAD_TO_DEG(angle1)));
  return {STEPS0, STEPS1};
}

void setup() {
  // set motor control pin modes
  pinMode(ARM0_M0_STEP, OUTPUT);
  arm0_m0.setMaxSpeed(1000.0);  // default
  arm0_m0.setAcceleration(100.0);  // default
  ..........
  pinMode(arm0.motor1_step_pin, OUTPUT);
  pinMode(arm0.motor0_dir_pin, OUTPUT);
  pinMode(arm0.motor1_dir_pin, OUTPUT);

  pinMode(arm1.motor0_step_pin, OUTPUT);
  pinMode(arm1.motor1_step_pin, OUTPUT);
  pinMode(arm1.motor0_dir_pin, OUTPUT);
  pinMode(arm1.motor1_dir_pin, OUTPUT);
  // digitalWrite(dirPin,HIGH); //Enables the motor to move in a particular direction

  all_motors.addStepper(arm0_m0);
  .............
}


void loop() {
  all_motors.moveTo(motor_positions);

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
