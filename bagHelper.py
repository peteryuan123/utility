import rosbag
import cv2
from cv_bridge import CvBridge
import os

# input
bag_file = '/home/mpl/data/Oxford/cloister/2021-12-02-10-19-05_1-cloister.bag'
topic_name = '/alphasense_driver_ros/cam1/compressed'

#output
folder = "/home/mpl/data/Oxford/cloister/cam1/" 

if(not os.path.exists(folder)):
    print("mkdir " + folder)
    os.makedirs(folder)

bag = rosbag.Bag(bag_file, "r")
bridge = CvBridge()

bag_data = bag.read_messages(topic_name)

for topic, msg, t in bag_data:
    cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    cv2.imwrite(folder + str(t)+ ".jpg", cv_image)
    print(t)
    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(0)
