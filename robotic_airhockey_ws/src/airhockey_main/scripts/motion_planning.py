import math
import numpy as np


class NoSolutionError(Exception):
    """Error when trying to solve for intersection between puck trajectory
    and perimeter of arm reach."""
    pass


def extrapolate_puck(puck_pose):
    """
    From current puck position and velocity, determine position when reaches
    perimeter of either arm's reach.

    :param puck_pose: (x,y) position and velocity of puck
    :type: Struct()

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


def find_goal_pos(arm_length, base_x, base_y, p):
    """
    Finds intersection location (x, y) as well as time between incoming
    puck and perimeter of arm's max reach. Derviation shown on project page.
    Must use a try and except block with this function since it raises errors.

    :param arm_length
    :type: float

    :param base_x: x position of base of arm
    :type: float

    :param base_y: y position of base of arm
    :type: float

    :param p: puck position and velocity information
    :type: Struct()

    :returns goal_x, goal_y: next goal location for arm
    :type: float

    :returns time_to_collision (milliseconds)
    :type: float

    """
    error_str = "Couldn't calculate new goal position for arm"
    # Quadratic formula
    a = (p.vx)**2 + (p.vy)**2
    b = ((2 * p.x * p.vx) - (2 * p.vx * base_x) +
         (2 * p.y * p.vy) - (2 * p.vy * base_y))
    c = (p.x - base_x)**2 + (p.y - base_y)**2 - 4 * arm_length**2
    sq_root_vals = b**2 - 4 * a * c
    if (sq_root_vals < 0) or (a == 0):
        raise NoSolutionError(error_str)
    else:
        time_to_collision = (-b - (sq_root_vals)**0.5) / (2 * a)
        if (time_to_collision < 0):
            time_to_collision = (-b + (sq_root_vals)**0.5) / (2 * a)
        goal_x = p.x + p.vx * time_to_collision
        goal_y = p.y + p.vy * time_to_collision
    if not (goal_x >= 0 and goal_y >= 0 and time_to_collision >= 0):
        raise NoSolutionError(error_str)
    return goal_x, goal_y, time_to_collision


