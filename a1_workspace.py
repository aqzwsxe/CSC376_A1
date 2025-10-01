import roboticstoolbox as rtb
import numpy as np
import roboticstoolbox as rtb
from spatialmath import *
from math import pi
import matplotlib.pyplot as plt
from matplotlib import cm
np.set_printoptions(linewidth=100, formatter={'float': lambda x: f"{x:8.4g}" if abs(x) > 1e-10 else f"{0:8.4g}"})

if __name__ == '__main__':

    # matplotlib widget

    L = [rtb.PrismaticMDH(theta=0.0, a=0.0, alpha=0.0, qlim=[0, 0.5]),
            rtb.RevoluteDH(alpha=pi/2, d=0, a=0, qlim=[-pi/2, pi/2]),
            rtb.PrismaticMDH(theta=0.0, a=0.0,  alpha=-pi/2, qlim=[0, 0.5])]
    counter = 0
    for i in L:
        print()
        robot = rtb.DHRobot(
            [
                i
            ], name="myRobot")
        robot.teach(robot.q)


    # Open a PyPlot with the teach panel
