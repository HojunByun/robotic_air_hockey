import math
import typing as t

def extrapolate_puck(puck_pose: dict):
    return

def pick_arm():
    is_first_arm = True
    return is_first_arm

def calc_joints_from_pos(arm_length: float,
                         goal_x: float,
                         goal_y: float) -> t.Tuple[float, float]:
    """
    Geometric solution to 2-DOF robot arm inverse kinematics.

    :param arm_length: length of robot arm (in our case, both links same length)

    :param goal_x: target x-position of end_effector

    :param goal_y: target y-position

    """
    theta1 = math.acos((goal_x**2 + goal_y**2 - 2*arm_length**2) /
                       (2*arm_length**2))
    theta0 = math.atan2(goal_y, goal_x) - (atan2(arm_length*math.sin(theta1),
                                                 arm_length + arm_length*math.cos))
    return (theta0, theta1)
