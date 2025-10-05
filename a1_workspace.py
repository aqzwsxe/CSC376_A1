import roboticstoolbox as rtb
import numpy as np
import roboticstoolbox as rtb
from spatialmath import *
from math import pi
import matplotlib.pyplot as plt
from matplotlib import cm

np.set_printoptions(linewidth=100, formatter={'float': lambda x: f"{x:8.4g}" if abs(x) > 1e-10 else f"{0:8.4g}"})

# By: Katherine Qin, Zehao Fan, Oct 2025
# References: Lecture slides, code from practical, 
# https://matplotlib.org/stable/tutorials/pyplot.html
# https://matplotlib.org/stable/api/pyplot_summary.html
# https://matplotlib.org/stable/api/figure_api.html

def plot_bounding_box(ax, total_points):
    if len(total_points) == 0:
        print("There's no points in the total_points")
        return
    # Convert each [x,y,z] to a numpy array
    points_array = np.array(total_points)
    min_coords = points_array.min(axis=0)
    max_coords = points_array.max(axis=0)

    xmin,ymin,zmin = min_coords
    xmax,ymax,zmax = max_coords

    # Print bounds for information
    print("\n--- Bounding Box Extents ---")
    print(f"X range: [{xmin:8.4g}, {xmax:8.4g}]")
    print(f"Y range: [{ymin:8.4g}, {ymax:8.4g}]")
    print(f"Z range: [{zmin:8.4g}, {zmax:8.4g}]")
    print(f"Volume: {(xmax - xmin) * (ymax - ymin) * (zmax - zmin):8.4g}")

    # Define the 8 vertices (corners) of the box
    corners = np.array([
        [xmin, ymin, zmin],  # 0
        [xmax, ymin, zmin],  # 1
        [xmax, ymax, zmin],  # 2
        [xmin, ymax, zmin],  # 3
        [xmin, ymin, zmax],  # 4
        [xmax, ymin, zmax],  # 5
        [xmax, ymax, zmax],  # 6
        [xmin, ymax, zmax]  # 7
    ])

    edges = [
        # Bottom plane
        (0, 1), (1, 2), (2, 3), (3, 0),
        # Top plane
        (4, 5), (5, 6), (6, 7), (7, 4),
        # Vertical edges
        (0, 4), (1, 5), (2, 6), (3, 7)
    ]

    for (i, j) in edges:
        xs = [corners[i, 0], corners[j, 0]]
        ys = [corners[i, 1], corners[j, 1]]
        zs = [corners[i, 2], corners[j, 2]]
        ax.plot(xs, ys, zs, color='r', linestyle='--', linewidth=1.5, alpha=0.8, label='Bounding Box' if i == 0 else "")

# if __name__ == '__main__':

# initialized the robot joints based on the values in the DH table
L = [rtb.PrismaticMDH(alpha=0, a=0, offset=2, theta=0, qlim=[0, 0.5]),
     rtb.RevoluteMDH(alpha=pi / 2, a=0, d=0, qlim=[-pi / 2, pi / 2]),
     rtb.PrismaticMDH(alpha=-pi / 2, a=0.0, offset=1, theta=0, qlim=[0, 0.5])]

# outputs the robot figure in robotics toolbox
robot = rtb.DHRobot(L, name="myRobot")

# opens a 3D model of the joints with the robot teach window
robot.teach(robot.q)

# Sample the robot’s workspace using the joint limits and the forward kinematics.
# We know that for joint1, qlim is range along d1.
# for joint2, qlim is range about theta2
# for joint3, qlim is range about d3
# therefore, we can sample random points for the end-effector positions (where the points are at)
size = 2000
np.random.seed(42)

# initialize random points using joint limits
d1 = np.random.uniform(0, 0.5, size)
theta2 = np.random.uniform(-pi / 2, pi / 2, size)
d3 = np.random.uniform(0, 0.5, size)

# q = [d1, theta2, d3]
# using np.column_stack to create a size x 3 array with col0 = d1, col1 = theta2, col2 = d3
samples = np.column_stack((d1, theta2, d3))
points = []

for q in samples:
    T = robot.fkine(q)  # calculate the SE3 matrix
    points.append(T.t)  # get the [x,y,z] points

x = []
y = []
z = []

# get all x, y and z values to plot in 3D graph
for i in range(size):
    x.append(points[i][0])  # get all x values
    y.append(points[i][1])  # get all y values
    z.append(points[i][2])  # get all z values

plt.plot(x, y, z, 'o')  # plots points as circles
plt.show()

# initialize a 10 x 8 3D figure
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# plot a point cloud using viridis colourmap and small circular dots
ax.scatter(x, y, z, c=z, cmap='viridis', marker='o', s=1, alpha=0.3, label="The Workspace")

# concatenate the point cloud with the zero config [0,0,0]
# calculate the max range along each axis to create a bounding box
all_coords = np.concatenate([np.array(points), [[0, 0, 0]]])
max_range = np.array([all_coords[:, 0].max() - all_coords[:, 0].min(),
                      all_coords[:, 1].max() - all_coords[:, 1].min(),
                      all_coords[:, 2].max() - all_coords[:, 2].min()]).max() / 2.0

# calculate the midpoint of each axis
mid_x = (all_coords[:, 0].max() + all_coords[:, 0].min()) * 0.5
mid_y = (all_coords[:, 1].max() + all_coords[:, 1].min()) * 0.5
mid_z = (all_coords[:, 2].max() + all_coords[:, 2].min()) * 0.5

# Set equal aspect ratio
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

# Add a legend for clarity (handle duplicate label for bounding box)
handles, labels = ax.get_legend_handles_labels()
unique_labels = dict(zip(labels, handles))
ax.legend(unique_labels.values(), unique_labels.keys())

plt.show()

# the points plotted shows the possible movements this robot can make
# the point cloud figures and bounding box will show up after closing the robot teach window.