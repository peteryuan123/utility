import rosbag
import cv2
from cv_bridge import CvBridge
import os
import sys
from scipy.spatial.transform import Rotation as sciRt
import numpy as np

if (len(sys.argv) != 4):
    print("usage: python3 preprocess.py bag_path")

# input
bag_file = sys.argv[1]

#output
# folder = "/home/mpl/data/sist/cam/" 
folder = "cam/"
if(not os.path.exists(folder)):
    print("mkdir " + folder)
    os.makedirs(folder)


bag = rosbag.Bag(bag_file, "r")
bridge = CvBridge()
img_data = bag.read_messages("/camera/rgb/image_mono/compressed")
odom_data = bag.read_messages("/odom")


with open("times.txt", mode="w") as file:
    for topic, msg, t in img_data:
        cv_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite(folder + str(t)+ ".jpg", cv_image)
        print(str(t), file=file)
        print(t)
    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(0)
file.close()

Qcb =  sciRt.from_matrix(np.array([[0, -1, 0],[0,0,-1], [1, 0, 0]]))
with open("odom.txt", mode="w") as file:
    for topic, msg, t in odom_data:

        Qbf = sciRt.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        tbf = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]).reshape(3, 1)

        Qcf = Qcb * Qbf * Qcb.inv()
        tcf = Qcb.as_matrix() @ tbf

        time = str(t)
        time_num = float(time)

        # print(tcf)
        msg_str = ""
        msg_str += (str(time_num/10e8) + " ")
        msg_str += (str(tcf[0][0]) + " ")
        msg_str += (str(tcf[1][0]) + " ")
        msg_str += (str(tcf[2][0]) + " ")
        msg_str += (str(Qcf.as_quat()[0]) + " ")
        msg_str += (str(Qcf.as_quat()[1]) + " ")
        msg_str += (str(Qcf.as_quat()[2]) + " ")
        msg_str += (str(Qcf.as_quat()[3]))

        # msg_str = ""
        # msg_str += (str(float(str(t))/10e8) + " ")
        # msg_str += (str(msg.pose.pose.position.x) + " ")
        # msg_str += (str(msg.pose.pose.position.y) + " ")
        # msg_str += (str(msg.pose.pose.position.z) + " ")
        # msg_str += (str(msg.pose.pose.orientation.x) + " ")
        # msg_str += (str(msg.pose.pose.orientation.y) + " ")
        # msg_str += (str(msg.pose.pose.orientation.z) + " ")
        # msg_str += (str(msg.pose.pose.orientation.w))
        print(msg_str, file=file)
file.close()