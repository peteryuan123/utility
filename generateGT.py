import rosbag
import cv2
from cv_bridge import CvBridge
import os
import sys
import numpy as np
from scipy.spatial.transform import Rotation as sciRt

if (len(sys.argv) != 4):
    print("usage: python3 generateGT.py [bag_path] [topic_name] [dest_file]")

# input
# bag_file = '/home/mpl/data/sist/2022-11-01-16-50-13.bag'
# topic_name = '/camera/rgb/image_mono'
bag_file = sys.argv[1]
topic_name = sys.argv[2]
file = sys.argv[3]

bag = rosbag.Bag(bag_file, "r")
bag_data = bag.read_messages(topic_name)

# Qcb =  sciRt.from_matrix(np.array([[0, -1, 0],[0,0,1], [-1, 0, 0]]))
# Qcb =  sciRt.from_matrix(np.array([[0, 1, 0],[0,0,-1], [-1, 0, 0]]))
Qcb =  sciRt.from_matrix(np.array([[0, -1, 0],[0,0,-1], [1, 0, 0]]))

init = False
Qwinit = sciRt.from_quat([0,0,0,1])
twinit = np.array([0,0,0]).reshape(3,1)

count = 0
with open(file, mode="w") as file:
    for topic, msg, t in bag_data:
        time = str(t)
        time_num = float(time)

        # if (count != 12):
        #     count += 1
        #     continue
        
        count = 0
        Qwc = sciRt.from_quat([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        twc = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]).reshape(3, 1)

        # Qinitc =  Qwinit.inv() *  Qwc
        # tinitc =  Qcb.as_matrix() @  Qwinit.inv().as_matrix() @ twc

        Qinitc =   Qcb * Qwc * Qcb.inv() 
        tinitc =   Qcb.as_matrix() @ twc

        # print(tcf)
        msg_str = ""
        msg_str += (str(time_num/10e8) + " ")
        msg_str += (str(tinitc[0][0]) + " ")
        msg_str += (str(tinitc[1][0]) + " ")
        msg_str += (str(tinitc[2][0]) + " ")
        msg_str += (str(Qinitc.as_quat()[0]) + " ")
        msg_str += (str(Qinitc.as_quat()[1]) + " ")
        msg_str += (str(Qinitc.as_quat()[2]) + " ")
        msg_str += (str(Qinitc.as_quat()[3]))


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

