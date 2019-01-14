#include <stdlib.h>
#include <math.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <airhockey_main/ArmAngles.h>

#include "arm_data_struct.h"

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
// set up subscriber(arduino to raspberry pi data)
ros::Subscriber<airhockey_main::ArmAngles> recalib_sub("recalib_arm_angles", &recalib_cb);

// set up publisher(arduino echoes the joint0 data it receives)
std_msgs::Float32 joint0;
ros::Publisher arm_echo_pub("arm_echo", &joint0);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // initialize values
  arm0.arm_theta0 = 0.0;
  arm0.arm_theta1 = 0.0;
  arm1.arm_theta0 = 0.0;
  arm1.arm_theta1 = 0.0;

  // notify ros master about subscriber and publisher
  nh.initNode();
  nh.subscribe(recalib_sub);
  nh.advertise(arm_echo_pub);
}


void loop() {
  joint0.data = arm0.arm_theta0;

  // publish new joint data
  arm_echo_pub.publish(&joint0);
  nh.spinOnce();
  delay(10);
}
