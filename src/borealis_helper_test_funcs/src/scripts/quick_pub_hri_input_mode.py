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
    output_state_list = ["Go_There", "Go_There", "Go_There"]
    output_yaw_list = [10.1, 20.5, 30]

    borealis_hri_output_msg.uav_pose_array = output_pose_array
    borealis_hri_output_msg.uav_state_list = output_state_list
    borealis_hri_output_msg.uav_yaw_list = output_yaw_list

    hri_output_publisher = rospy.Publisher(hri_output_topic, Borealis_HRI_Output, queue_size=10)
    
    rospy.spin()