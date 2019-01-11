#!/usr/bin/env python

import rospy
from airhockey_main.msg import ArmAngles

import motion_planning as motion

# import some opencv scripts

def publish_arm_data(publisher, arm_data):
    """
    Takes in all necessary arm_data and publishes it to arduino. Serves as main
    communication node with arduino regardless of purpose.

    :param publisher: ROS publisher to send arm data to arduino
    :type: rospy.Publisher

    :param arm_data: arm data
    :type: dict

    """
    arm_data = ArmAngles()
    arm_data.success = arm_data['success']
    arm_data.is_goal = arm_data['is_goal']
    arm_data.arm0_joint0 = arm_data['arm0_joint0']
    arm_data.arm0_joint1 = arm_data['arm0_joint1']
    arm_data.arm1_joint0 = arm_data['arm1_joint0']
    arm_data.arm1_joint1 = arm_data['arm1_joint1']
    publisher.publish(arm_data)

def defense_mode(publisher, arm_data, puck_pose):
    """
    Defense mode picks the best arm to use for a given situation avoid a
    collision between both arms. Then simply commands that arm to move to the
    anticipated puck location without any regard for puck's recoil velocity.
    Mainly serves for testing inverse kinematics since it's no fun to play
    against this.

    :param publisher: ROS publisher to send arm data to arduino
    :type: rospy.Publisher

    :param arm_data: holds arm data, use of info, reliability bool
    :type: dict

    :param puck_pose: holds current puck position and velocity x,y
    :type: dict

    """
    arm_data['sucess'] = True
    arm_data['is_goal'] = True
    (goal_x, goal_y) = motion.extrapolate_puck(puck_pose)
    is_first_arm = motion.pick_arm() # provide arm position and desired location
    theta0, theta1 = motion.calc_joints_from_pos(arm_data['length'],
                                                 goal_x, goal_y)
    if (is_first_arm):
        arm_data['arm0_joint0'] = theta0
        arm_data['arm0_joint1'] = theta1
    else:
        arm_data['arm1_joint0'] = theta0
        arm_data['arm1_joint1'] = theta1
    publish_arm_data(publisher, arm_data)

def attack_mode():
    return

def hybrid_mode():
    """
    Maybe a hybrid between attack and defense mode. Have the arm first slow down
    the puck, then recoil and hit the puck.

    """
    return

def collab_mode():
    """
    """
    return

# MAIN LOOP that runs on raspberry pi
def run_main():
    arm_data_pub = rospy.Publisher('arm_angles_topic', ArmAngles,
                                    queue_size=10)
    rospy.init_node('arm_data_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    score = {'human': 0,
             'robot': 0}
    arm_data = {'success': false,
                'is_goal': false,
                'arm0_joint0': 0.0,
                'arm0_joint1': 0.0,
                'arm1_joint0': 0.0,
                'arm1_joint1': 0.0}

    while not rospy.is_shutdown():
        # try to implement some distribution with random chance of selecting a
        # mode
        if (score['human'] < 3):
            defense_mode()
        elif (3 <= score['human'] > 6):
            attack_mode()
        else:
            collab_mode()
        rate.sleep()  # delay by 1/10 sec

if __name__ == '__main__':
    try:
        run_main()
    except rospy.ROSInterruptException:
        pass

