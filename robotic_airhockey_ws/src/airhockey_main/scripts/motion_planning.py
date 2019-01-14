import math
import numpy as np


class NoSolutionError(Exception):
    """Error when trying to solve for intersection between puck trajectory
    and perimeter of arm reach."""
    pass


def calc_joints_from_pos(arm_L, goal_x, goal_y):
    """
    Geometric solution to 2-DOF robot arm inverse kinematics.
    NOTE: goal_x and goal_y must be definied WITH RESPECT TO BASE OF ARM, so
    provide something like (arm_L, goal_x - base_x, goal_y - base_y)

    :param arm_L: length of robot arm (in our case, both links same length)
    :type: float

    :param goal_x: target x-position of end_effector
    :type: float

    :param goal_y: target y-position
    :type: float

    :returns (theta0, theta1): two joint angles required for goal position
    :type: tuple(float, float)

    """
    # while desired x, y is out of reach of arm
    # check if hypotenuse of triangle formed by x and y is > combined arm length
    theta1 = math.acos((goal_x**2 + goal_y**2 - 2*arm_L**2) /
                       (2*arm_L**2))
    theta0 = math.atan2(goal_y, goal_x) - (
                math.atan2(arm_L*math.sin(theta1),
                           arm_L + arm_L*math.cos(theta1)))
    # simply invert conversion to get radians to degree
    return (theta0, theta1)


def find_goal_pos(arm_L, base_x, base_y, p):
    """
    Finds intersection location (x, y) as well as time between incoming
    puck and perimeter of arm's max reach. Derviation shown on project page.
    Use a try-except block for both arms, pick the first one that has a valid
    solution.

    :param arm_L
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
    c = (p.x - base_x)**2 + (p.y - base_y)**2 - 4 * arm_L**2
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

    # double-check that goal is within reach of arm
    assert(((goal_x - base_x)**2 + (goal_y - base_y)**2)**0.5 < 2*arm_L)
    return goal_x, goal_y, time_to_collision


def calc_goal_joint_pose(arm_L, arm_speed, table_L,
                         theta0, theta1, goal_x, goal_y):
    """
    Calculates the desired velocity of arm when it reaches the goal position.
    Also calculates angular accelerations for both joints for them to reach
    end position at desired angle.

    :param arm_L: length of one link
    :type: float

    :param table_L: length of table
    :type: float

    :param goal_x, goal_y: goal x, y position of end effector
    :type: float

    """
    theta_g = math.atan2(L - goal_y, goal_x)
    arm_vel_g = np.array([
        [arm_speed * math.cos(theta_g)],
        [arm_speed * math.sin(theta_g)]
    ])
    theta0_g, theta1_g = calc_joints_from_pos(arm_L, goal_x, goal_y)

    # determine angular velocity of joints
    jacobian = np.array([
        [-arm_L * math.sin(theta1) - arm_L * math.sin(theta0 + theta1),
         -arm_L * math.sin(theta0 + theta1)],
        [arm_L * math.cos(theta1) + arm_L * math.cos(theta0 + theta1),
         arm_L * math.cos(theta0 + theta1)]
    ])
    inv_jacobian = np.linalg.inv(jacobian)
    omega = np.matmul(inv_jacobian, arm_vel_g)
    del_theta0 = theta0_g - theta0
    del_theta1 = theta1_g - theta1
    alpha0 = omega[0]**2 / (2 * del_theta0)
    alpha1 = omega[1]**2 / (2 * del_theta1)


    # HAVE SOME WHILE LOOP THAT CONSTANTLY BRINGS COLLISION POINT CLOSER
    # UNTIL DESIRED OMEGA AND ALPHA ARE WITHIN LIMITS OF MOTORS B/C
    # IF TRY TO SET TOO HIGH VEL OR ACC, ARM WILL JUST MISS PUCK


    return omega[0], omega[1], alpha0, alpha1


def linearize_trajectory(table_length, table_width, puck_pose, extent_of_check):
    """
    Determine the collisions of the puck with the sides and find the final
    velocity and fake position of the puck as if the puck were only moving in
    a straight line at that velocity at same distance as currently is.

    :param puck_pose: contains info about orig puck's position and velocity
    :type: Struct()DEG_TO_RAD

    :param extent_of_check: scaling factor that scales the length of the table
    down and checks for all x-collisions up until that point. For example,
    maybe 1/2 of table length because after that the arms can hopefully reach,
    simplifying both arms' perimeters to straight line across table. NOTE: This
    is with robot side as the start of table, so 1/4 * length gives quarter way
    of table starting from robot side.
    :type: float

    :returns: (x_f, y_f, vx_f, vy_f, time_to_reach)
    After filtering out collisions with walls,
    this is the approximate pose of the puck as if it were only moving in a
    straight line from current position ignoring walls.


    """
    # should already be positive b/c negative velocity, but use abs() for safety
    time_to_reach = ((extent_of_check * table_length - puck_pose.y) /
                     (puck_pose.vy))
    total_x_dist = puck_pose.x + puck_pose.vx * time_to_reach
    num_wall_collisions = total_x_dist // table_width
    if (num_wall_collisions % 2 == 0):
        x_f = total_x_dist - num_wall_collisions * table_width
        vx_f = puck_pose.vx
    else:
        x_f = table_width - (total_x_dist - num_wall_collisions * table_width)
        vx_f = -puck_pose.vx
    # check to ensure math is all correct: puck should reach inside table
    # when in range of arms
    assert(0 <= total_x_dist + vx_f * time_to_reach <= table_width)
    return (total_x_dist, puck_pose.y, vx_f, puck_pose.vy, time_to_reach)


