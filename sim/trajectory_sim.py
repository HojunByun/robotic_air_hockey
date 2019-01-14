from tkinter import *
import math
import sys

import motion_planning as motion  # actual functions driving robot movement

# Credit to the 15-112 Course at Carnegie Mellon University for providing
# example graphics functions with tkinter
# Link: https://www.cs.cmu.edu/~112/notes/notes-animations-part2.html
# this script demonstrates the correct determination of puck velocity vector
# intersection with bounds of robot arm

SEC_TO_MILLIS = float(1)/1000
DEG_TO_RAD = math.pi/180
SC_WIDTH = 200
SC_HEIGHT = 400
EXTENT_CHECK = float(3)/4  # note: reversed for graphics, -> 1 means closer to arms
root = Tk()
class Struct(object): pass


## Drawing ##
def init(data):
    # CONSTANTS
    # arm starts at bottom center of screen, perspective of robot
    # note in graphics, y must decrease for arm to point upwards
    data.arm0_x = SC_WIDTH/2
    data.arm0_y = SC_HEIGHT
    data.ball_r = 10
    data.dot_r = 1
    data.ball_orig_x = SC_WIDTH/2  # starting position of ball
    data.ball_orig_y = data.ball_r
    data.ball_vel = 5
    data.arm_length = 50
    data.arm_width = 5
    data.timerDelay = 25  # 25ms delay between each updated frame
    data.arm_color = "red"
    data.ball_color = "yellow"
    data.dot_color = "purple"
    data.real_arm_bounds_color = "orange"
    data.appr_arm_bounds_color = "green"

    # VARIABLES
    # so all angles in degrees so can increment/decrement easily
    data.arm0_theta0 = 0
    data.arm0_theta1 = 0
    data.goal0 = 45
    data.goal1 = 0
    data.ball_x = data.ball_orig_x  # actual location of ball, center
    data.ball_y = data.ball_orig_y
    data.ball_goal_x = 0
    data.ball_goal_y = 0
    approx_vel(data)  # modifies data.ball_vx, data.ball_vy
    data.collision_x, data.collision_y = 0, 0
    data.pred_x, data.pred_y = 0, 0
    data.pred_vx, data.pred_vy = 0, 0
    data.reach_bound = 0
    data.time_to_collision = 0
    data.reached_goal = False
    data.move_ball = False


def mousePressed(event, data):
    data.ball_goal_x = event.x
    data.ball_goal_y = event.y
    approx_vel(data)
    predict_puck_motion(data)


def timerFired(data):
    if data.move_ball:
        if data.ball_x + data.ball_r > data.width:
            data.ball_vx *= -1  # bounce with opposite velocity
        else:
            data.ball_x += data.ball_vx
        if data.ball_y + data.ball_r > SC_HEIGHT:
            data.ball_x = data.ball_origin_x
            data.ball_y = data.ball_origin_y
            data.move_ball = False
        else:
            data.ball_y += data.ball_vy
    if not data.reached_goal:
        move_arm(data)


def keyPressed(event, data):
    # press Q or q to quit
    if (event.char == 'q') or (event.char == 'Q'):
        root.destroy()
        sys.exit()
    elif (event.char == 'l') or (event.char == 'L'):
        data.move_ball = not data.move_ball


def draw_arm(canvas, data):
    end_x0 = data.arm0_x + data.arm_length * math.cos(data.arm0_theta0 * DEG_TO_RAD)
    end_y0 = data.arm0_y - data.arm_length * math.sin(data.arm0_theta0 * DEG_TO_RAD)

    # start of second link is the end of first link
    # note that the overall angle for the second joint is the combined angle of
    # the first joint and second joint
    end_x1 = end_x0 + data.arm_length * (
                math.cos((data.arm0_theta0 + data.arm0_theta1) * DEG_TO_RAD))
    end_y1 = end_y0 - data.arm_length * (
                math.sin((data.arm0_theta0 + data.arm0_theta1) * DEG_TO_RAD))

    #draw first link
    canvas.create_line(data.arm0_x, data.arm0_y,  # starting x, y
                       end_x0, end_y0,            # ending x, y
                       fill=data.arm_color,           # color
                       width=data.arm_width)      # width of line

    # draw second link
    canvas.create_line(end_x0, end_y0,
                       end_x1, end_y1,
                       fill=data.arm_color, width=data.arm_width)


def draw_ball_and_collision(canvas, data):
    canvas.create_oval(data.ball_x - data.ball_r, data.ball_y - data.ball_r,
                       data.ball_x + data.ball_r, data.ball_y + data.ball_r,
                       fill=data.ball_color)
    canvas.create_oval(data.collision_x - data.dot_r,
                       data.collision_y - data.dot_r,
                       data.collision_x + data.dot_r,
                       data.collision_y + data.dot_r,
                       fill=data.dot_color)


def draw_perimeter(canvas, data):
    canvas.create_oval(data.arm0_x - 2 * data.arm_length,
                       data.arm0_y - 2 * data.arm_length,
                       data.arm0_x + 2 * data.arm_length,
                       data.arm0_y + 2 * data.arm_length,
                       outline=data.real_arm_bounds_color)


def draw_approx_bound(canvas, data):
    canvas.create_line(0, EXTENT_CHECK * SC_HEIGHT,
                       SC_WIDTH, EXTENT_CHECK * SC_HEIGHT,
                       fill=data.appr_arm_bounds_color, width=5)


def draw_trajectory(canvas, data):
    canvas.create_line(data.pred_x, data.pred_y,
                        data.pred_x + data.pred_vx * data.reach_bound,
                        data.pred_y + data.pred_vy * data.reach_bound,
                        fill="red", width = 5)


def redrawAll(canvas, data):
    draw_arm(canvas, data)
    draw_perimeter(canvas, data)
    draw_approx_bound(canvas, data)
    draw_trajectory(canvas, data)
    draw_ball_and_collision(canvas, data)
    canvas.create_text(data.width/2, 50,
                       text="Each Arm Length: %d" % data.arm_length)
    canvas.create_text(data.width/2, data.height/2,
                       text="theta0: %f" % data.arm0_theta0)
    canvas.create_text(data.width/2, data.height/2 + 50,
                       text="theta1: %f" % data.arm0_theta1)


def run(width=300, height=300):
    def redrawAllWrapper(canvas, data):
        canvas.delete(ALL)
        canvas.create_rectangle(0, 0, data.width, data.height,
                                fill='white', width=0)
        redrawAll(canvas, data)
        canvas.update()

    def timerFiredWrapper(canvas, data):
        timerFired(data)
        redrawAllWrapper(canvas, data)
        # pause, then call timerFired again
        canvas.after(data.timerDelay, timerFiredWrapper, canvas, data)

    def keyPressedWrapper(event, canvas, data):
        keyPressed(event, data)
        redrawAllWrapper(canvas, data)

    def mousePressedWrapper(event, canvas, data):
        mousePressed(event, data)
        redrawAllWrapper(canvas, data)

    # Set up data and call init
    data = Struct()
    data.width = width
    data.height = height
    root.resizable(width=False, height=False) # prevents resizing window
    init(data)
    # create the root and the canvas
    canvas = Canvas(root, width=data.width, height=data.height)
    canvas.configure(bd=0, highlightthickness=0)
    canvas.pack()
    root.bind("<Key>", lambda event:
        keyPressedWrapper(event, canvas, data))
    root.bind("<Button-1>", lambda event:
        mousePressedWrapper(event, canvas, data))
    timerFiredWrapper(canvas, data)
    # and launch the app
    print('Press q to quit')
    root.mainloop()  # blocks until window is closed
    print("bye!")


## Logic and Calculations ##
def approx_vel(data):
    dist = ((data.ball_goal_x - data.ball_orig_x)**2 +
            (data.ball_goal_y - data.ball_orig_y)**2)**0.5
    # mult by -1 to account for graphics coordinate frame
    ang = math.atan2((data.ball_goal_y - data.ball_orig_y),
                     data.ball_goal_x - data.ball_orig_x)

    print('x', 'orig x')
    print(data.ball_goal_x, data.ball_orig_x)
    print('y', 'orig y')
    print(data.ball_goal_y, data.ball_orig_y)
    print('Approx angle: ', ang / DEG_TO_RAD)
    print('dist: ', dist)
    print('vx: ', dist * math.cos(ang))
    print('vy: ', dist * math.sin(ang))

    data.ball_vx = data.ball_vel * math.cos(ang) * SEC_TO_MILLIS / data.timerDelay
    # -1 to account for graphics inversion of coordinate frame
    data.ball_vy = data.ball_vel * math.sin(ang) * SEC_TO_MILLIS / data.timerDelay


def predict_puck_motion(data):
    puck_pose = Struct()
    # note: graphics frame v.s real-world frame flipped on y-axis
    puck_pose.y = data.ball_y
    puck_pose.x = data.ball_x
    puck_pose.vx = data.ball_vx
    puck_pose.vy = data.ball_vy

    print("Orig(cartesian): puck_pose.x, puck_pose.y, puck_pose.vx, puck_pose.vy")
    print(puck_pose.x, puck_pose.y, puck_pose.vx, puck_pose.vy)

    puck_pose.x, puck_pose.y, puck_pose.vx, puck_pose.vy, data.reach_bound = (
            motion.linearize_trajectory(SC_HEIGHT, SC_WIDTH, puck_pose, EXTENT_CHECK))

    print("New: puck_pose.x, puck_pose.y, puck_pose.vx, puck_pose.vy")
    print(puck_pose.x, puck_pose.y, puck_pose.vx, puck_pose.vy)

    data.pred_x = puck_pose.x
    data.pred_y = puck_pose.y
    data.pred_vx = puck_pose.vx
    data.pred_vy = puck_pose.vy

    data.collision_x, collision_y, data.time_to_collision = (
            motion.find_goal_pos(data.arm_length,
                                 data.arm0_x, data.arm0_y,
                                 puck_pose))
    data.collision_y = collision_y

    print("collision_x, collision_y", data.collision_x, data.collision_y)
    joint0, joint1 = motion.calc_joints_from_pos(data.arm_length,
                        data.collision_x - data.arm0_x,
                        -1 * (data.collision_y - data.arm0_y))
    data.goal0 = joint0 / DEG_TO_RAD
    data.goal1 = joint1 / DEG_TO_RAD
    data.reached_goal = False


def check_reached_goal(cur, goal):
    # error less than 1 deg is good enough
    return abs(cur - goal) < 1


def move_arm(data):
    # Note: there is an illusion sometimes that only one arm moves at a time
    # but when both joint angles have opposite change (one dec, the other inc)
    # and the second link will look like it's staying in place
    # joint0
    reached_goal0 = check_reached_goal(data.arm0_theta0, data.goal0)
    reached_goal1 = check_reached_goal(data.arm0_theta1, data.goal1)
    if not reached_goal0:
        if (data.arm0_theta0 > data.goal0):
            data.arm0_theta0 -= 1
        elif (data.arm0_theta0 < data.goal0):
            data.arm0_theta0 += 1

    # joint1
    if not reached_goal1:
        if (data.arm0_theta1 > data.goal1):
            data.arm0_theta1 -= 1
        elif (data.arm0_theta1 < data.goal1):
            data.arm0_theta1 += 1

    # finally reached goal when both joints have reached their goal angles
    data.reached_goal = reached_goal0 and reached_goal1


run(SC_WIDTH*2, SC_HEIGHT*2)
