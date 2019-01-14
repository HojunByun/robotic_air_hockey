from tkinter import *
import math
import sys

# Credit to the 15-112 Course at Carnegie Mellon University for providing
# example graphics functions with tkinter
# Link: https://www.cs.cmu.edu/~112/notes/notes-animations-part2.html

DEG_TO_RAD = math.pi/180
SC_WIDTH = 200
SC_HEIGHT = 400
root = Tk()

## Drawing ##
def init(data):
    # CONSTANTS
    # arm starts at bottom center of screen, perspective of robot
    # note in graphics, y must decrease for arm to point upwards
    data.arm0_x = SC_WIDTH/2
    data.arm0_y = SC_HEIGHT
    data.length = 50
    data.arm_width = 5
    data.color = "red"

    # VARIABLES
    # so all angles in degrees so can increment/decrement easily
    data.arm0_theta0 = 0
    data.arm0_theta1 = 0
    data.goal0 = 45
    data.goal1 = 0
    data.reached_goal = False

    data.timerDelay = 25  # 25ms delay between each updated frame


def timerFired(data):
    # once reach goal, take in new user input
    if (data.reached_goal):
        take_input(data)
    # else, keep moving arm until reach goal
    else:
        move_arm(data)

def keyPressed(event, data):
    # press Q or q to quit
    if (event.char == 'q') or (event.char == 'Q'):
        root.destroy()
        sys.exit()


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


def draw_arm(canvas, data):
    end_x0 = data.arm0_x + data.length * math.cos(data.arm0_theta0 * DEG_TO_RAD)
    end_y0 = data.arm0_y - data.length * math.sin(data.arm0_theta0 * DEG_TO_RAD)

    # start of second link is the end of first link
    # note that the overall angle for the second joint is the combined angle of
    # the first joint and second joint
    end_x1 = end_x0 + data.length * (
                math.cos((data.arm0_theta0 + data.arm0_theta1) * DEG_TO_RAD))
    end_y1 = end_y0 - data.length * (
                math.sin((data.arm0_theta0 + data.arm0_theta1) * DEG_TO_RAD))

    #draw first link
    canvas.create_line(data.arm0_x, data.arm0_y,  # starting x, y
                       end_x0, end_y0,            # ending x, y
                       fill=data.color,           # color
                       width=data.arm_width)      # width of line

    # draw second link
    canvas.create_line(end_x0, end_y0,
                       end_x1, end_y1,
                       fill=data.color, width=data.arm_width)


def redrawAll(canvas, data):
    draw_arm(canvas, data)
    canvas.create_text(data.width/2, 50,
                       text="Each Arm Length: %d" % data.length)
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
    # Set up data and call init
    class Struct(object): pass
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
    timerFiredWrapper(canvas, data)
    # and launch the app
    root.mainloop()  # blocks until window is closed
    print("bye!")


def get_float_input():
    try:
        val = float(input())
        return val
    except:
        root.destroy()
        sys.exit()


## Logic and Calculations ##
def calc_joints_from_pos(arm_length, goal_x, goal_y):
    # while desired x, y is out of reach of arm
    # check if hypotenuse of triangle formed by x and y is > combined arm length
    while (goal_x**2 + goal_y**2)**0.5 > (2*arm_length):
        print("Out of Range, Try Again:")
        print("Goal X:")
        goal_x = get_float_input()
        print("Goal Y:")
        goal_y = get_float_input()
    theta1 = math.acos((goal_x**2 + goal_y**2 - 2*arm_length**2) /
                       (2*arm_length**2))
    theta0 = math.atan2(goal_y, goal_x) - (
            math.atan2(arm_length*math.sin(theta1),
                       arm_length + arm_length*math.cos(theta1)))
    # simply invert conversion to get radians to degree
    return (theta0 / DEG_TO_RAD, theta1 / DEG_TO_RAD)


def take_input(data):
    print("Goal X:")
    goal_x =  get_float_input()
    print("Goal Y:")
    goal_y = get_float_input()
    (data.goal0, data.goal1) = calc_joints_from_pos(data.length, goal_x, goal_y)
    data.reached_goal = False  # not necessarily False, but allows for reset


def check_reached_goal(cur, goal):
    # error less than 1 deg is good enough
    return abs(cur - goal) < 1


run(SC_WIDTH, SC_HEIGHT)
