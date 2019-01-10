#include "ros/ros.h"
#include "airhockey_main/ArmAngles.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_pub_arm_angles");
    ros::NodeHandle nh;
    ros::Publisher chatter_pub = (
      nh.advertise<airhockey_main::ArmAngles>("recalib_arm_angles", 1000));
    airhockey_main::ArmAngles angles;
    angles.arm0_joint0 = 0;
    angles.arm0_joint1 = 0;
    angles.arm1_joint0 = 0;
    angles.arm1_joint1 = 0;
    ros::Rate loop_rate(10);
    float count = 0;
    while (ros::ok())
    {
        angles.arm0_joint0 = count;
        angles.success = (count > 100);
        ROS_INFO("%f", count);
        chatter_pub.publish(angles);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }

    return 0;
}
