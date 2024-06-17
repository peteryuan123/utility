import sys
import numpy as np
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf import transformations as trans
import rospy

def main():
    odom_path = sys.argv[1]
    odom = np.loadtxt(odom_path)


    odom_pub = rospy.Publisher("/odom", PoseStamped, queue_size=100)
    path_pub = rospy.Publisher("/path", Path, queue_size=10)
    rospy.init_node('pubPath')

    # path = Path()
    # path.header.stamp = rospy.Time.from_sec(odom[0][0])
    # path.header.frame_id = "map"
    # for i in range(odom.shape[0]):
    #     cur_pose = PoseStamped()
    #     cur_pose.header.frame_id = "map"

    #     cur_pose.header.stamp = rospy.Time.from_sec(odom[i][0])
    #     cur_pose.pose.position.x = odom[i][1]
    #     cur_pose.pose.position.y = odom[i][2]
    #     cur_pose.pose.position.z = odom[i][3]
    #     cur_pose.pose.orientation.x = odom[i][4]
    #     cur_pose.pose.orientation.y = odom[i][5]
    #     cur_pose.pose.orientation.z = odom[i][6]
    #     cur_pose.pose.orientation.w = odom[i][7]
    #     path.poses.append(cur_pose)

    # while(True):
    #     path_pub.publish(path)

    for i in range(odom.shape[0]):
        cur_pose = PoseStamped()
        cur_pose.header.frame_id = "map"
        cur_pose.header.stamp = rospy.Time.from_sec(odom[i][0])
        cur_pose.pose.position.x = odom[i][1]
        cur_pose.pose.position.y = odom[i][2]
        cur_pose.pose.position.z = odom[i][3]

        r_matrix = trans.quaternion_matrix(odom[i][4:8])
        r_matrix = r_matrix @ np.array([[0, 1, 0, 0],
                                        [0, 0, 1, 0],
                                        [1, 0, 0, 0],
                                        [0, 0, 0, 1]])
        
        r_quaternion = trans.quaternion_from_matrix(r_matrix)

        cur_pose.pose.orientation.x = r_quaternion[0]
        cur_pose.pose.orientation.y = r_quaternion[1]
        cur_pose.pose.orientation.z = r_quaternion[2]
        cur_pose.pose.orientation.w = r_quaternion[3]
        
        print(cur_pose.header.stamp)
        odom_pub.publish(cur_pose)

        if (i != odom.shape[0]):
            sleep_time = odom[i+1][0] - odom[i][0]
            sleep_duration = rospy.Duration.from_sec(sleep_time)
            rospy.sleep(sleep_duration)

if __name__ == "__main__":
    main()
