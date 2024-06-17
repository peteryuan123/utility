import rosbag
import cv2
from cv_bridge import CvBridge
import os
import sys

if (len(sys.argv) != 4):
    print("usage: python3 bagHelper.py [bag_path] [topic_name] [dest_folder]")

# input
# bag_file = '/home/mpl/data/sist/2022-11-01-16-50-13.bag'
# topic_name = '/camera/rgb/image_mono'
bag_file = sys.argv[1]
topic_name = sys.argv[2]



#output
# folder = "/home/mpl/data/sist/cam/" 
folder = sys.argv[3]
if(not os.path.exists(folder)):
    print("mkdir " + folder)
    os.makedirs(folder)

bag = rosbag.Bag(bag_file, "r")
bridge = CvBridge()
bag_data = bag.read_messages(topic_name)

if (topic_name.split("/")[-1] == "compressed"):
    compressed = True
else:
    compressed = False

for topic, msg, t in bag_data:
    if (compressed):
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
    else:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imwrite(folder + str(t)+ ".jpg", cv_image)
    print(t)
    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(0)
