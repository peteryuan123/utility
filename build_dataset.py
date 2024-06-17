import rospy
import rosbag
from cv_bridge import CvBridge
import cv2
import os
from geometry_msgs.msg import PoseStamped
import sys

def save_depth_image(msg, output_folder, bag_name):
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(msg)
    depth_filename = os.path.join(output_folder, 'depth', "{:.6f}.png".format(msg.header.stamp.to_sec()))
    cv2.imwrite(depth_filename, depth_image)
    # with open(os.path.join(output_folder, 'depth.txt'), 'a') as f:
    #     # f.write("# depth maps\n")
    #     # f.write("# file: '{}'\n".format(bag_name))
    #     # f.write("# timestamp filename\n")
    #     f.write("{:.6f} depth/{}.png\n".format(msg.header.stamp.to_sec(), os.path.basename(depth_filename)))

def save_rgb_image(msg, output_folder, bag_name):
    bridge = CvBridge()
    rgb_image = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
    rgb_filename = os.path.join(output_folder, 'rgb', "{:.6f}.png".format(msg.header.stamp.to_sec()))
    cv2.imwrite(rgb_filename, rgb_image)
    # with open(os.path.join(output_folder, 'rgb.txt'), 'a') as f:
    #     # f.write("# color images\n")
    #     # f.write("# file: '{}'\n".format(bag_name))
    #     # f.write("# timestamp filename\n")
    #     f.write("{:.6f} rgb/{}.png\n".format(msg.header.stamp.to_sec(), os.path.basename(rgb_filename)))

def save_imu_data(msg, output_folder, bag_name):
    imu_filename = os.path.join(output_folder, 'accelerometer.txt')
    with open(imu_filename, 'a') as f:
        if os.stat(imu_filename).st_size == 0:
            f.write("# accelerometer data\n")
            f.write("# file: '{}'\n".format(bag_name))
            f.write("# timestamp ax ay az\n")
        f.write("{:.6f} {} {} {}\n".format(msg.header.stamp.to_sec(), msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))

def save_groundtruth_data(msg, output_folder, bag_name):
    groundtruth_filename = os.path.join(output_folder, 'groundtruth.txt')
    with open(groundtruth_filename, 'a') as f:
        if os.stat(groundtruth_filename).st_size == 0:
            f.write("# ground truth trajectory\n")
            f.write("# file: '{}'\n".format(bag_name))
            f.write("# timestamp tx ty tz qx qy qz qw\n")
        f.write("{:.6f} {} {} {} {} {} {} {}\n".format(msg.header.stamp.to_sec(), msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))

def process_bag(input_bag, output_folder):
    bag_name = os.path.basename(input_bag)
    if not os.path.exists(output_folder):
        os.makedirs(os.path.join(output_folder, 'depth'))
        os.makedirs(os.path.join(output_folder, 'rgb'))

    i = 0
    with rosbag.Bag(input_bag, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            print(f"Processed msgs count:{i}")
            if topic == "/camera/left/depth_image_undistort":
                save_depth_image(msg, output_folder, bag_name)
            elif topic == "/camera/left/image_mono_undistort":
                save_rgb_image(msg, output_folder, bag_name)
            elif topic == "/gt/pose":
                save_groundtruth_data(msg, output_folder, bag_name)
            i += 1
if __name__ == "__main__":
    input_bag = sys.argv[1]
    output_folder = sys.argv[2]
    process_bag(input_bag, output_folder)
