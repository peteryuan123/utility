import numpy as np
from scipy.spatial.transform import Rotation as sciR
from scipy import linalg
import matplotlib.axes as axes
import matplotlib.pyplot as plt


def plotCoordinateSystem(axis, linewidth, pose, color):

    r = pose[0].as_matrix()
    axis_x = pose[1].reshape(3) + 0.5 * r[:, 0]
    axis_y = pose[1].reshape(3) + 0.5 * r[:, 1]
    axis_z = pose[1].reshape(3) + 0.5 * r[:, 2]
    # X-axis
    axis.plot([pose[1][0], axis_x[0]], [pose[1][1], axis_x[1]], zs=[pose[1][2], axis_x[2]], color = "red", linewidth=linewidth)
    axis.plot([pose[1][0], axis_y[0]], [pose[1][1], axis_y[1]], zs=[pose[1][2], axis_y[2]], color = "green", linewidth=linewidth)
    axis.plot([pose[1][0], axis_z[0]], [pose[1][1], axis_z[1]], zs=[pose[1][2], axis_z[2]], color = "blue", linewidth=linewidth)


def main():
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    poses = []
    poses.append([sciR.from_quat([0, 0, 0, 1]), np.array([0, 0, 0])])
    # poses.append([sciR.from_quat([-0.0035513000780663484, -0.7041772154795547, -0.71001521560788838, 0.00048070001056697365]), \
    #                  np.array([-0.0087408998928222102, 0.05844015674045925, -0.0086801525401812366])])

    # poses.append([sciR.from_quat([0.7078268167375279, -0.00085320002017507507, -0.0043314001024218479, 0.70637221670313199]), \
    #                  np.array([-0.0070045779540184785, -0.071625650217341727, -0.011119423288663392])])

    # poses.append([sciR.from_matrix([[-0.99996328, -0.0063136 ,  0.00579492 ],
    #                                 [ 0.00579066,  0.00069323,  0.99998299],
    #                                 [-0.00631751, 0.99997983, -0.00065664]]), \
    #                  np.array([-0.00674352,  0.01084157, -0.06125465])])
    poses.append([sciR.from_quat([1, 0, 0, 0]), \
                     np.array([0.038, -0.008, 0.065])])             
    for pose in poses:
        plotCoordinateSystem(ax, 1, pose, "blue")
    
    
    plt.show()

if __name__ == "__main__":
    main()