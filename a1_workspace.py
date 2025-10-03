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

    # L = [rtb.PrismaticMDH(alpha=0, a=0, offset=2, theta=0, qlim=[0, 0.5]),
    #         rtb.RevoluteMDH(alpha=pi/2, a=0, d=0, qlim=[-pi/2, pi/2]),
    #         rtb.PrismaticMDH(alpha=-pi/2, a=0.0, offset=1, theta=0, qlim=[0, 0.5])]
    L = [rtb.PrismaticMDH(a=0.0, alpha=0.0, qlim=[2, 2.5]),
            rtb.RevoluteMDH(alpha=pi/2,a=0, d=0,  qlim=[-pi/2, pi/2]),
            rtb.PrismaticMDH(a=0.0,  alpha=-pi/2,  qlim=[1, 1.5])]
    counter = 0

    # outputs the robot figure in robotics toolbox
    robot = rtb.DHRobot(L, name="myRobot") 
    robot.plot([0,0,0]) # zero config at theta = [0,0,0]

    robot.teach(robot.q)

    # Sample the robot’s workspace using the joint limits and the forward kinematics. 
    # We know that for joint1, qlim is range along d1. 
    # for joint2, qlim is range about theta2
    # for joint3, qlim is range about d3
    # therefore, we sample random points for the end-effector positions (where the points are at)
    size = 50 

    # initialize random points using joint limits
    d1 = np.random.uniform(0, 0.5, size)
    theta2 = np.random.uniform(-pi/2, pi/2, size)
    d3 = np.random.uniform(0, 0.5, size)
    
    #q = [d1, theta2, d3]
    #using np.column_stack to create a size x 3 array with col0 = d1, col1 = theta2, col2 = d3
    samples = np.column_stack((d1, theta2, d3))
    points = []

    for q in samples:
        T = robot.fkine(q) # calculate the SE3 matrix
        points.append(T.t) # get the [x,y,z] points 

    x = []
    y = []
    z = []

    # get all x, y and z values to plot in 3D graph 
    for i in range(size):
        x.append(points[i][0]) # get all x values
        y.append(points[i][1]) # get all y values
        z.append(points[i][2]) # get all z values

    plt.plot(x, y, z, 'o') # plots points as circles
    plt.show()
    
    # the points plotted shows the possible movements this robot can make

    # Which visualization do you think will work better? the above one outputs one figure with three joints
    # and the one below outputs 3 different figures for each joint
    # i'll leave them both here for reference

    #for i in L:
        #print()
        #robot = rtb.DHRobot(
            #[
                #i
            #], name="myRobot")
        #robot.teach(robot.q)


    # Open a PyPlot with the teach panel
