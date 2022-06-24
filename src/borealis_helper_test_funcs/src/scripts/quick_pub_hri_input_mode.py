#!/usr/bin/env python

## Publishes a topic with the the with/after tf transforms
  
import rospy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int32, Float64, String
from borealis_hri_msgs.msg import Borealis_HRI_Output
import random

if __name__ == '__main__':
    rospy.init_node('Pub_new_input')
    hri_output_topic = "/borealis_hri_output_topic"
    borealis_hri_output_msg = Borealis_HRI_Output()
    output_pose_array = PoseArray()

    go_there_poses = Pose()

    go_there_poses.position.x = 0
    go_there_poses.position.y = 1
    go_there_poses.position.z = 2
    go_there_poses.orientation.x = 0
    go_there_poses.orientation.y = 0
    go_there_poses.orientation.z = 0
    go_there_poses.orientation.w = 1

    output_pose_array.poses.append(go_there_poses)

    output_state_list = ["Go_There", "Nil", "Nil"]
    output_yaw_list = [10.1, 20.5, 30]

    borealis_hri_output_msg.uav_pose_array = output_pose_array
    borealis_hri_output_msg.uav_state_list = output_state_list
    borealis_hri_output_msg.uav_yaw_list = output_yaw_list

    hri_output_publisher = rospy.Publisher(hri_output_topic, Borealis_HRI_Output, queue_size=10)

    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():

        borealis_hri_output_msg = Borealis_HRI_Output()
        output_pose_array = PoseArray()

        go_there_poses = Pose()
        go_there_poses1 = Pose()
        go_there_poses2 = Pose()

        go_there_poses.position.x = 4 + 2 
        go_there_poses.position.y = 1
        go_there_poses.position.z = 2
        go_there_poses.orientation.x = 0
        go_there_poses.orientation.y = 0
        go_there_poses.orientation.z = 0
        go_there_poses.orientation.w = 1

        output_pose_array.poses.append(go_there_poses)

        go_there_poses1.position.x = 3 + 1
        go_there_poses1.position.y = 1
        go_there_poses1.position.z = 2
        go_there_poses1.orientation.x = 0
        go_there_poses1.orientation.y = 0
        go_there_poses1.orientation.z = 0
        go_there_poses1.orientation.w = 1
        output_pose_array.poses.append(go_there_poses1)

        go_there_poses2.position.x = 2 
        go_there_poses2.position.y = 1
        go_there_poses2.position.z = 2
        go_there_poses2.orientation.x = 0
        go_there_poses2.orientation.y = 0
        go_there_poses2.orientation.z = 0
        go_there_poses2.orientation.w = 1
        output_pose_array.poses.append(go_there_poses2)

        borealis_hri_output_msg.uav_pose_array = output_pose_array
        borealis_hri_output_msg.uav_state_list = output_state_list
        borealis_hri_output_msg.uav_yaw_list = output_yaw_list

        hri_output_publisher.publish(borealis_hri_output_msg)
        rate.sleep()
        print("Done")