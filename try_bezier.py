import numpy as np
import bezier
from matplotlib import pyplot as plt

"""
    One approach to solving the trajectory motion plan is to first determine
    the desired starting angle. Maybe just take the vector that is
    orthogonal to the displacement vector between the two (x, y) points and
    use the angle of this orthog. The ending angle is just the angle of
    the desired end velocity (atan(vy, vx)). Then to ensure these two angle
    preconditions are met, generate two other points for the linear
    projections from the two orig points. Specifically, first determine
    distance between the orig points. Then do scalar projection from the
    starting and ending points using this distance, the starting and ending
    angles, and some scalar < 1 (like 1/4), then use these 4 points to
    generate bezier curve and use the curve.plot() with num_pts = 10 for
    approximation to generate waypoints for arm.

"""
if __name__ == '__main__':
    nodes1 = np.asfortranarray([
        [0.0, 0.5, 1.0],  # x points
        [0.0, 1.0, 0.0]   # y points
    ])

    curve1 = bezier.Curve(nodes1, degree=2)
    ax = curve1.plot(num_pts=256)
    lines = ax.plot(
            nodes1[0, :], nodes1[1, :],
            marker="o", linestyle="None", color='black')
    plt.show()
