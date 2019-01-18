import copy
import math
import numpy as np

import utilities as util


def predict_puck_motion(table, arm, puck_pose):
    """
    This is the main function that will be called by the raspberry pi
    constantly. The raspberry pi will call computer vision scripts to return the
    puck's current location and velocity. This script calls many other functions
    and overall returns information such as the final collision of the puck
    with the extent of reach of the arm, all the deflections of the puck with
    the wall, and the desired joint angles for the robot arm.

    :param table:
    :type: Struct that contains the attributes:
        width: width of table
        length: length of table

    :param arm:
    :type: Struct that contains:
        x: base x of arm
        y: base y of arm
        link_length: length of one link(two links for our 2DOF robot arm)
        num_links: number of links for one arm (in our case 2)

    :param puck_pose:
    :type: Struct:
        x: center x of puck
        y: center y of puck
        vx: velocity x-component of puck
        vy: velocity y-component of puck

    :returns:
    """
    # note: graphics frame v.s real-world frame flipped on y-axis
    transformed_pose = copy.deepcopy(puck_pose)
    transformed_pose.y = util.transform(transformed_pose.y, True, table.length)
    transformed_pose.vy = util.transform(transformed_pose.vy, False)

    deflections = find_deflections(table, arm, transformed_pose, [])

    lin_trajectory = linearize_trajectory(puck_pose, deflections)

    collision_info = vector_circle_intersect(arm, lin_trajectory)

    joint_info = util.Struct()
    joint0, joint1 = calc_joints_from_pos(arm.link_length,
                        collision_info.x - arm.x,
                        (collision_info.y - arm.y))
    joint_info.joint0 = joint0
    joint_info.joint1 = joint1

    return collision_info, deflections, joint_info


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
    try:
        theta1 = math.acos((goal_x**2 + goal_y**2 - 2*arm_L**2) /
                       (2*arm_L**2))
    except:
        # floating point error where distance of goal from base of arm is only
        # slightly greater than reach of arm (ie: 100.000000000003)
        # solution: make arm slighty larger, almost same accuracy so good enough
        arm_L *= 1.01
        theta1 = math.acos((goal_x**2 + goal_y**2 - 2*arm_L**2) /
                       (2*arm_L**2))
    theta0 = math.atan2(goal_y, goal_x) - (
                math.atan2(arm_L*math.sin(theta1),
                           arm_L + arm_L*math.cos(theta1)))
    # simply invert conversion to get radians to degree
    return (theta0, theta1)


def vector_circle_intersect(arm, p):
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
    arm_L = arm.link_length * arm.num_links
    a = (p.vx)**2 + (p.vy)**2
    b = ((2 * p.x * p.vx) - (2 * p.vx * arm.x) +
         (2 * p.y * p.vy) - (2 * p.vy * arm.y))
    c = (p.x - arm.x)**2 + (p.y - arm.y)**2 - arm_L**2
    sq_root_vals = b**2 - 4 * a * c

    if (sq_root_vals < 0) or (a == 0):
        raise util.NoSolutionError(error_str)
    else:
        time_to_collision = (-b - (sq_root_vals)**0.5) / (2 * a)
        if (time_to_collision < 0):
            time_to_collision = (-b + (sq_root_vals)**0.5) / (2 * a)
        goal_x = p.x + p.vx * time_to_collision
        goal_y = p.y + p.vy * time_to_collision

    if not (goal_x >= 0 and goal_y >= 0 and time_to_collision >= 0):
        raise util.NoSolutionError(error_str)

    # double-check that goal is within reach of arm
    # assert(((goal_x - base_x)**2 + (goal_y - base_y)**2)**0.5 < 2*arm_L)
    collision_info = util.Struct()
    collision_info.x = goal_x
    collision_info.y = goal_y
    collision_info.time_to_collision = time_to_collision

    return collision_info


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
    theta_g = math.atan2(arm_L - goal_y, goal_x)
    arm_vel_g = np.array([
        [arm_speed * math.cos(theta_g)],
        [arm_speed * math.sin(theta_g)]
    ])
    theta0_g, theta1_g = calc_joints_from_pos(arm_L, goal_x, goal_y)

    # determine angular velocity of joints
     # determine angular velocity of joints
    jacobian = np.array([
        [-arm_L * math.sin(theta1) - arm_L * math.sin(theta0 + theta1),     -arm_L * math.sin(theta0 + theta1)],
        [arm_L * math.cos(theta1) + arm_L * math.cos(theta0 + theta1),      arm_L * math.cos(theta0 + theta1)]
    ])

    jacobian1 = np.array([
        [arm_L * math.cos(theta0) * math.cos(theta1) - arm_L * math.sin(theta0) * math.sin(theta1), -arm_L * math.cos(theta0) * math.sin(theta1) - arm_L * math.sin(theta0) * math.cos(theta1)],
        [arm_L * math.sin(theta0) * math.cos(theta1) + arm_L * math.cos(theta0) * math.sin(theta1), -arm_L * math.sin(theta0) * math.sin(theta1) + arm_L * math.cos(theta0) * math.cos(theta1)]
    ])

    jacobian2 = np.array([
        [(-arm_L * math.sin(theta0) - arm_L *(math.sin(theta0) * math.cos(theta1) + math.cos(theta0) * math.sin(theta1))), 
        (-arm_L * math.cos(theta0) * math.sin(theta1) - arm_L * math.sin(theta0) * math.cos(theta1))],
        [( arm_L * math.cos(theta0) + arm_L *(math.cos(theta0) * math.cos(theta1) - math.sin(theta0) * math.sin(theta1))),
        (-arm_L * math.sin(theta0) * math.sin(theta1) + arm_L * math.cos(theta0) * math.cos(theta1))]
    ])



    inv_jacobian = np.linalg.inv(jacobian2)
    omega = np.matmul(inv_jacobian, arm_vel_g)
    del_theta0 = theta0_g - theta0
    del_theta1 = theta1_g - theta1
    alpha0 = omega[0]**2 / (2 * del_theta0)
    alpha1 = omega[1]**2 / (2 * del_theta1)


    # HAVE SOME WHILE LOOP THAT CONSTANTLY BRINGS COLLISION POINT CLOSER
    # UNTIL DESIRED OMEGA AND ALPHA ARE WITHIN LIMITS OF MOTORS B/C
    # IF TRY TO SET TOO HIGH VEL OR ACC, ARM WILL JUST MISS PUCK


    return omega[0], omega[1], alpha0, alpha1


def find_deflections(table, arm,
                    puck_pose, deflections=[]):
    """
    Calculates the full trajectory of a puck including all its different
    deflections from wall sides. NOTE: modifies the puck_pose, so need to
    pass in a deep copy at the start. NOTE: requires the cartesian coordinate
    frame, so need to transform graphics coordinates to cartesian.
    Cases:
        - puck bounces off wall like normal
        - puck never bounces off wall before reaching arm
        - puck reaches final bounce location in which case:
            - its distance from arm base is within reach of arm
            - OR its next deflection_y is beyond the table bounds
    :param deflections: list of tuples that contain the (x, y, vx, vy, time) of
    deflection
    """
    p_copy = copy.deepcopy(puck_pose)
    reach_radius = arm.link_length * arm.num_links
    assert(0 <= p_copy.x <= table.width)
    # (vx == 0) implies puck moving straight down, no wall deflections
    if p_copy.vx == 0:  # avoid division by zero error first
        return deflections
    elif p_copy.vx > 0:  # moving right
        assert(table.width > p_copy.x)
        time_deflection = (table.width - p_copy.x) / p_copy.vx
        p_copy.x = table.width  # next x position right after deflection
    else:  #   moving left
        time_deflection = (0 - p_copy.x) / p_copy.vx
        p_copy.x = 0
    p_copy.vx *= -1
    assert(time_deflection > 0)
    p_copy.y = p_copy.y + p_copy.vy * time_deflection

    # angle too shallow, will never collide with wall before reaching arm
    if p_copy.y < arm.y:
        return deflections
    dist_from_base = util.distance(arm.x, arm.y, p_copy.x, p_copy.y)
    # no need to transform or keep, previous pose will lead puck to arms
    if dist_from_base <= reach_radius:
        return deflections
    if len(deflections) > 0: prev_time = deflections[-1][4]
    else: prev_time = 0
    try:
        collision = vector_circle_intersect(arm, puck_pose)
        deflections.append([collision.x, collision.y,
                            puck_pose.vx, puck_pose.vy,
                            prev_time + collision.time_to_collision])
        return deflections
    except util.NoSolutionError:
        # -1 to account for collided puck, simply have puck move in opposite dir
        deflections.append([p_copy.x, p_copy.y,
                        p_copy.vx, p_copy.vy,
                        prev_time + time_deflection])
        return find_deflections(table, arm, p_copy, deflections)


def linearize_trajectory(puck_pose, deflections):
    if len(deflections) == 0:
        # no need for transformations, current pose will guide puck to arms
        return copy.deepcopy(puck_pose)
    else:
        # use last collision and treat as if constant velocity entire time
        # (x, y, vx, vy, time to reach position)
        new_p = util.Struct()
        last_collision = deflections[-1]
        last_x, last_y = last_collision[0], last_collision[1]
        last_vx, last_vy = last_collision[2], last_collision[3]
        total_time = last_collision[4]
        # x_0 = x_f - vx * t,  solve for linear trajectory with last velocity
        new_p.x = last_x - last_vx * total_time
        new_p.y = last_y - last_vy * total_time
        new_p.vx, new_p.vy = last_vx, last_vy
        return new_p


def linearize_trajectory_v1(table_length, table_width, puck_pose, extent_of_check):
    """
    NOTE: DO NOT USE THIS FUNCTION. DOES NOT WORK. Simply historical reference.
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
    collisions = find_collisions(table_width, table_length,
                                 copy.deepcopy(puck_pose),
                                 num_wall_collisions)
    if (num_wall_collisions % 2 == 0):
        x_f = total_x_dist - num_wall_collisions * table_width
        vx_f = puck_pose.vx
    else:
        x_f = table_width - (total_x_dist - num_wall_collisions * table_width)
        vx_f = -puck_pose.vx
    # check to ensure math is all correct: puck should reach inside table
    # when in range of arms
    # assert(0 <= total_x_dist + vx_f * time_to_reach <= table_width)
    return (total_x_dist, puck_pose.y, vx_f, puck_pose.vy, time_to_reach)

