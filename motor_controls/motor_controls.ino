#include <stdlib.h>
#include <math.h>
#include <ros.h>
#include <airhockey_main/ArmAngles.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#include "serial_inputs.h"

#define LENGTH (double)5  // inches
#define SQ(x) (double)(x*x)
#define STEPS_PER_ROT (float)3200
#define RAD_TO_STEP(x) x*(STEPS_PER_ROT/(2*PI))

// ros setup
ros::NodeHandle nh;

// arm1(left) and arm2(right) setup
AccelStepper arm0_stepper0(AccelStepper::DRIVER, 2, 3);
AccelStepper arm0_stepper1(AccelStepper::DRIVER, 4, 5);
AccelStepper arm1_stepper0(AccelStepper::DRIVER, 6, 7);
AccelStepper arm1_stepper1(AccelStepper::DRIVER, 8, 9);

// globals
bool calibrated = false;

// callback that receives ground-truth arm angles from raspberry pi
void recalib_cb(airhockey_main::ArmAngles& arm_data)
{
  if (arm_data.success) {
    arm0_stepper0.setCurrentPosition(RAD_TO_STEP(arm_data.arm0_joint0));
    arm0_stepper1.setCurrentPosition(RAD_TO_STEP(arm_data.arm0_joint1));
    arm1_stepper0.setCurrentPosition(RAD_TO_STEP(arm_data.arm1_joint0));
    arm1_stepper1.setCurrentPosition(RAD_TO_STEP(arm_data.arm1_joint1));
    calibrated = true;
  } else {
    // some warning LED or display that says calibration incomplete
    // maybe force user to recalibrate by pressing some button again
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void receive_goal_cb(airhockey_main::ArmAngles& arm_data)
{
  arm0_stepper0.moveTo(RAD_TO_STEP(arm_data.arm0_joint0));
  arm0_stepper1.moveTo(RAD_TO_STEP(arm_data.arm0_joint1));
  arm1_stepper0.moveTo(RAD_TO_STEP(arm_data.arm1_joint0));
  arm1_stepper1.moveTo(RAD_TO_STEP(arm_data.arm1_joint1));
}

void wait_for_calib()
{
  // show some LED or text on an LCD screen notifying players
  digitalWrite(LED_BUILTIN, HIGH);
  while(!calibrated) ;  // check if this loop slows down Arduino
  digitalWrite(LED_BUILTIN, LOW);
}

// set default max speed and acceleration for the four motors
void config_motors()
{
  arm0_stepper0.setMaxSpeed(10000);  // max 10,000 steps/s
  arm0_stepper0.setAcceleration(5000);
  arm0_stepper1.setMaxSpeed(10000);
  arm0_stepper1.setAcceleration(5000);
  arm1_stepper0.setMaxSpeed(10000);
  arm1_stepper0.setAcceleration(5000);
  arm1_stepper1.setMaxSpeed(10000);
  arm1_stepper1.setAcceleration(5000);
}

ros::Subscriber<airhockey_main::ArmAngles> \
  recalib_sub("recalib_arm_angles", &recalib_cb);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  nh.initNode();
  nh.subscribe(recalib_sub);

  config_motors();
  wait_for_calib();
}


void loop() {
  nh.spinOnce();

  delay(100);

}
