from numpy import double
from pip import main
import glob, sys
from ros import rosbag
from ros import roslib
import rospy

roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import cv2
from cv_bridge import CvBridge

import time, os

cv_bridge = CvBridge()

def addImuMsg(bag, imudata):
    print("writing IMU..., len(%d)", len(imudata))
    for data in imudata:
        time_stamp = rospy.Time.from_sec(data[0])

        imu_msg = Imu()
        imu_msg.angular_velocity.x = data[1] 
        imu_msg.angular_velocity.y = data[2] 
        imu_msg.angular_velocity.z = data[3] 
        imu_msg.linear_acceleration.x = data[4] 
        imu_msg.linear_acceleration.y = data[5] 
        imu_msg.linear_acceleration.z = data[6] 
        imu_msg.header.stamp = time_stamp
        imu_msg.header.frame_id = "imu"
        
        bag.write("imu", imu_msg, time_stamp)
        print(data)
    return bag

def addImgMsg(bag, imgdata):
    print("writing img..., len(%d)", len(imgdata))
    for data in imgdata:
        time_stamp = rospy.Time.from_sec(data[0])

        image = cv2.imread(data[1])
        img_msg = cv_bridge.cv2_to_imgmsg(image)
        img_msg.header.stamp = time_stamp
        img_msg.header.frame_id = "camera"
        img_msg.width = image.shape[1]
        img_msg.height = image.shape[0]
        img_msg.encoding = "bgr8"

        bag.write("camera/image_raw", img_msg, time_stamp)
        print(data)

    return bag

def getAllImagesAndTimes(dir, ext):
    filenames = glob.glob(dir + "*." + ext)
    filenames.sort()

    imgdata = []
    for file_name in filenames:
        time_str = file_name.split("/")[-1].split(".")[0]
        time = double(time_str) / 10e8
        imgdata.append((time, file_name))

    return imgdata # [(time,name), (time, name), ...]

def getAllImuAndTimes(file_path): 
    datas = []
    with open(file_path, 'r') as f:
        lines = f.readlines()
        for line in lines:
            if (line[0] == "#"): 
                continue 
            else:
                data = line.split()
                if (len(data) == 7):
                    data = [float(i) for i in data]
                    data[0] /= 10e8
                    datas.append(data)

    f.close()
    return datas # [(time, w.x w.y w.z a.x a.y a.z), (time, w.x w.y w.z a.x a.y a.z), ... ]

def main():
    if (len(sys.argv) != 3):
        print("usage: python3 bagwriter.py [img_folder] [imu_file]")
        return
    img_folder = sys.argv[1]
    imu_file = sys.argv[2]

    img_data = getAllImagesAndTimes(img_folder, "jpg")
    imu_data = getAllImuAndTimes(imu_file)

    bag = rosbag.Bag("data.bag", 'w')
    bag = addImgMsg(bag, img_data)
    bag = addImuMsg(bag, imu_data)
    bag.close()

if __name__ == "__main__":
    main()