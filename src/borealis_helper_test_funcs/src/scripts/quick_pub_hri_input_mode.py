#!/usr/bin/env python

## Publishes a topic with the the with/after tf transforms
  
import rospy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int32, Float64, String
from borealis_hri_msgs.msg import Borealis_HRI_Output


if __name__ == '__main__':
    rospy.init_node('Pub_new_input')
    hri_output_topic = "/borealis_hri_output_topic"
    borealis_hri_output_msg = Borealis_HRI_Output()
    output_pose_array = PoseArray()

    go_there_posestamped = PoseStamped()

    go_there_posestamped.pose.position.x = 0
    go_there_posestamped.pose.position.y = 1
    go_there_posestamped.pose.position.z = 2
    go_there_posestamped.pose.orientation.x = 0
    go_there_posestamped.pose.orientation.y = 0
    go_there_posestamped.pose.orientation.z = 0
    go_there_posestamped.pose.orientation.w = 1

    output_pose_array.poses.append(go_there_posestamped)

    output_state_list = ["Go_There", "Go_There", "Go_There"]
    output_yaw_list = [10.1, 20.5, 30]

    borealis_hri_output_msg.uav_pose_array = output_pose_array
    borealis_hri_output_msg.uav_state_list = output_state_list
    borealis_hri_output_msg.uav_yaw_list = output_yaw_list

    hri_output_publisher = rospy.Publisher(hri_output_topic, Borealis_HRI_Output, queue_size=10)
    hri_output_publisher.publish(borealis_hri_output_msg)
    print("Done")

    rospy.spin()