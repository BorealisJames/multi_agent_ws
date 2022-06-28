#!/usr/bin/env python

## Publishes a topic with the the with/after tf transforms
  
import string
import rospy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int32, Float64, String, Bool, Int8
from borealis_hri_msgs.msg import Borealis_HRI_Output
import random

if __name__ == '__main__':
    rospy.init_node('Pub_new_input')
    hri_output_topic = "/borealis_hri_output_topic"

    uav1_hri_mode_pose_topic = "/uav1/hri_mode"
    uav2_hri_mode_pose_topic = "/uav2/hri_mode"

    uav1_input_pose_toppic = "/uav1/input_pose_stamped"
    uav2_input_pose_toppic = "/uav2/input_pose_stamped"

    uav1_number_of_agents_in_team = "/uav1/number_of_agents_in_team"
    uav2_number_of_agents_in_team = "/uav2/number_of_agents_in_team"

    uav_1_activate_planner_topic = "/uav1/teaming_planner/activate_planner"
    uav_2_activate_planner_topic = "/uav2/teaming_planner/activate_planner"

    uav1_input_pose_publisher = rospy.Publisher(uav1_input_pose_toppic, PoseStamped, queue_size=10)
    uav2_input_pose_publisher = rospy.Publisher(uav2_input_pose_toppic, PoseStamped, queue_size=10)

    uav1_number_of_agents_publisher = rospy.Publisher(uav1_number_of_agents_in_team, Int8, queue_size=10)
    uav2_number_of_agents_publisher = rospy.Publisher(uav2_number_of_agents_in_team, Int8, queue_size=10)

    uav1_activate_planner_publisher = rospy.Publisher(uav_1_activate_planner_topic, Bool, queue_size=10)
    uav2_activate_planner_publisher = rospy.Publisher(uav_2_activate_planner_topic, Bool, queue_size=10)

    uav1_hri_mode_pose_topic_publisher = rospy.Publisher(uav1_hri_mode_pose_topic, String, queue_size=10)
    uav2_hri_mode_pose_topic_publisher = rospy.Publisher(uav2_hri_mode_pose_topic, String, queue_size=10)

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

        pose_stamped = PoseStamped()

        go_there_poses = Pose()
        go_there_poses1 = Pose()
        go_there_poses2 = Pose()

        bool_to_send = True

        go_there_to_send = String("Go_There")
        follow_me_to_send = String("Follow_Me")

        go_there_poses.position.x = 3
        go_there_poses.position.y = -3
        go_there_poses.position.z = 1.3
        go_there_poses.orientation.x = 0
        go_there_poses.orientation.y = 0
        go_there_poses.orientation.z = 0
        go_there_poses.orientation.w = 1

        go_there_poses1.position.x = 3
        go_there_poses1.position.y = -3
        go_there_poses1.position.z = 1.3
        go_there_poses1.orientation.x = 0
        go_there_poses1.orientation.y = 0
        go_there_poses1.orientation.z = 0
        go_there_poses1.orientation.w = 1

        go_there_poses2.position.x = 3
        go_there_poses2.position.y = -3
        go_there_poses2.position.z = 1.3
        go_there_poses2.orientation.x = 0
        go_there_poses2.orientation.y = 0
        go_there_poses2.orientation.z = 0
        go_there_poses2.orientation.w = 1

        pose_stamped.pose = go_there_poses
        pose_stamped.header.stamp = rospy.Time.now()

        uav1_input_pose_publisher.publish(pose_stamped)
        uav2_input_pose_publisher.publish(pose_stamped)

        uav1_activate_planner_publisher.publish(bool_to_send)
        uav2_activate_planner_publisher.publish(bool_to_send)

        uav1_number_of_agents_publisher.publish(2)
        uav2_number_of_agents_publisher.publish(2)

        uav1_hri_mode_pose_topic_publisher.publish(follow_me_to_send)
        uav2_hri_mode_pose_topic_publisher.publish(follow_me_to_send)

        rate.sleep()
        print("Done")