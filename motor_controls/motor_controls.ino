#include <stdlib.h>
#include <math.h>
#include <ros.h>
#include <airhockey_main/ArmAngles.h>

#include "serial_inputs.h"
#include "arm_data_struct.h"

#define LENGTH (double)5  // inches
#define SQ(x) (double)(x*x)
#define STEP_ROT DEG_TO_RAD*3  // 3 degrees

// ros setup
ros::NodeHandle nh;

// arm0(left) and arm1(right) setup
store_arm_data arm0;
store_arm_data arm1;

// callback that receives ground-truth arm angles from raspberry pi
void recalib_cb(airhockey_main::ArmAngles& arm_data)
{
  if (arm_data.success) {
    arm0.arm_theta0 = arm_data.arm0_joint0;
    arm0.arm_theta1 = arm_data.arm0_joint1;
    arm1.arm_theta0 = arm_data.arm1_joint0;
    arm1.arm_theta1 = arm_data.arm1_joint1;
  } else {
    // some warning LED or display that says calibration incomplete
    // maybe force user to recalibrate by pressing some button again
    digitalWrite(LED_BUILTIN, 1 - digitalRead(LED_BUILTIN));
  }
}

ros::Subscriber<airhockey_main::ArmAngles> recalib_sub("recalib_arm_angles", &recalib_cb);

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

  pinMode(LED_BUILTIN, OUTPUT);
  arm0.arm_theta0 = 0.0;
  arm0.arm_theta1 = 0.0;
  arm1.arm_theta0 = 0.0;
  arm1.arm_theta1 = 0.0;
  nh.initNode();
  nh.subscribe(recalib_sub);

}


void loop() {
  nh.spinOnce();
  delay(100);

}
