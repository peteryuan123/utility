import rospy
import rosbag
from cv_bridge import CvBridge
import cv2
import os
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d
import sys

K = np.array([[886.19107, 0, 610.57891],
     [0, 886.59163, 514.59271],
     [0, 0, 1]])

Rbc = np.array([[-0.8571370239765715 ,  0.01322063096422758, -0.5149187674240417 , 
                0.03276713258773899, -0.9962462506036182 , -0.08012317505073686, 
               -0.514045170340666  , -0.08554895133864117,  0.853486344222504  ]]).reshape(3, 3)

tbc = np.array([ 0.06315837246891948, -0.02785306005411673, 0.0704789883105976]).reshape(3, 1)

Rcb = Rbc.T
tcb = -Rcb @ tbc


def get_camera_pt(color_img, depth_img):
    depth_img = depth_img / 1000
    valid_pts = np.logical_and(depth_img > 0, depth_img < 5)
    pix_x, pix_y = np.where(valid_pts)
    pix_z = depth_img[pix_x, pix_y]
    color = color_img[pix_x, pix_y]
    pix_y = (pix_y - K[0, 2]) * pix_z / K[0, 0]
    pix_x = (pix_x - K[1, 2]) * pix_z / K[1, 1]
    pix_xyz = np.concatenate((np.expand_dims(pix_y, axis=1), np.expand_dims(pix_x, axis=1), np.expand_dims(pix_z, axis=1)), axis=1)
    return pix_xyz, color

def main():
    path = sys.argv[1]
    info_file = sys.argv[2]

    time_list = []
    gt_list = []
    f = open(info_file)
    line = f.readline()
    while line:
        temp_info = line.split(" ")
        time_list.append(temp_info[0])
        twb = np.array([float(temp_info[1]), float(temp_info[2]), float(temp_info[3])])
        Rwb = R.from_quat([float(temp_info[4]), float(temp_info[5]), float(temp_info[6]), float(temp_info[7])]).as_matrix()
        gt_list.append((Rwb, twb))
        line = f.readline()
    f.close()

    RrefB_w = gt_list[0][0].T
    trefB_w = -gt_list[0][0].T @ gt_list[0][1].reshape(3, 1)
    for i in range(len(time_list)):
        time = time_list[i]
        Rwb, twb = gt_list[i]

        print(os.path.join(path, "rgb", time + ".png"))
        color_img = cv2.imread(os.path.join(path, "rgb", time + ".png"))
        depth_img = cv2.imread(os.path.join(path, "depth", time + ".png"), cv2.IMREAD_UNCHANGED)
        cloud, color = get_camera_pt(color_img, depth_img)
        if (i != 0):
            cloud = Rcb @ (RrefB_w @ (Rwb @( Rbc @ cloud.T + tbc) + twb.reshape(3, 1)) + trefB_w) + tcb.reshape(3, 1)
            cloud = cloud.T
        point_cloud_o3d = o3d.geometry.PointCloud()
        point_cloud_o3d.points = o3d.utility.Vector3dVector(cloud)
        point_cloud_o3d.colors = o3d.utility.Vector3dVector(color / 255)
        o3d.visualization.draw_geometries([point_cloud_o3d])
        o3d.io.write_point_cloud(os.path.join(path, "point_cloud", time + ".xyzrgb"), point_cloud_o3d, write_ascii=True)

    # bag_name = os.path.basename(input_bag)
    # rgb_list, depth_list, time_list = [], [], []
    # gt_dict = {}
    # bag = rosbag.Bag(input_bag, 'r')
    # depth_topic = bag.read_messages("/camera/left/depth_image_undistort")
    # rgb_topic = bag.read_messages("/camera/left/image_mono_undistort")
    # gt_topic = bag.read_messages("/gt/pose")
    # bridge = CvBridge()

    # counter = 0
    # print("depth..")
    # for topic, msg, t in depth_topic:
    #     if (counter % 6 == 0):
    #         depth_image = bridge.imgmsg_to_cv2(msg, '32FC1')
    #         depth_list.append(depth_image)
    #         time_list.append(t)
    #         print(t)
    #     counter += 1

    # counter = 0
    # print("rgb..")
    # for topic, msg, t in rgb_topic:
    #     if (counter % 6 == 0):
    #         rgb_image = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
    #         rgb_list.append(rgb_image)
    #     counter += 1

    # print("gt..")
    # for topic, msg, t in gt_topic:
    #     Rwb = R.from_quat([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]).as_matrix()
    #     twb = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]).reshape(3, 1)
    #     gt_dict[t] = (Rwb, twb)

    # global_cloud = o3d.geometry.PointCloud()
    # for i in range(len(rgb_list)):
    #     cur_cloud, color = get_camera_pt(rgb_list[i], depth_list[i])
    #     time = time_list[i]
    #     if time not in gt_dict.keys():
    #         continue
    #     Rwb, twb = gt_dict[time]
    #     cur_cloud = Rwb @ (Rbc @ cur_cloud.T + tbc) + twb
    #     point_cloud_o3d = o3d.geometry.PointCloud()
    #     point_cloud_o3d.points = o3d.utility.Vector3dVector(cur_cloud.T)
    #     point_cloud_o3d.colors = o3d.utility.Vector3dVector(color / 255)
    #     global_cloud += point_cloud_o3d
    # o3d.visualization.draw_geometries([global_cloud])


if __name__ == "__main__":
    main()