import math

def extrapolate_puck(puck_pose):
    """
    From current puck position and velocity, determine position when reaches
    perimeter of either arm's reach.

    :param puck_pose: (x,y) position and velocity of puck
    :type: dict

    """
    return

def pick_arm():
    is_first_arm = True
    return is_first_arm

def calc_joints_from_pos(arm_length, goal_x, goal_y):
    """
    Geometric solution to 2-DOF robot arm inverse kinematics.

    :param arm_length: length of robot arm (in our case, both links same length)
    :type: float

    :param goal_x: target x-position of end_effector
    :type: float

    :param goal_y: target y-position
    :type: float

    :returns (theta0, theta1): two joint angles required for goal position
    :type: tuple(float, float)

    """
    theta1 = math.acos((goal_x**2 + goal_y**2 - 2*arm_length**2) /
                       (2*arm_length**2))
    theta0 = math.atan2(goal_y, goal_x) - (atan2(arm_length*math.sin(theta1),
                                                 arm_length + arm_length*math.cos))
    return (theta0, theta1)
