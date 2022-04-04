#!/usr/bin/env python

"""
This is a topic renamer script, the idea of this script is to rename ros topics by rebroadcasting them to a diff topic name 
or piping different topic publisher to another type of publisher.
Stop gap solution if there is a difference in naming convention and renaming them aint easy.
Currently this is used for Borealis project to perform system check on hardware.
Do in gazebo sim -> rosbag sensor topic names of interest -> find actual hardware -> rosbag play to stimulate fake sensors...
-> rename any of the topic using this script -> simulate mix real time sensor input vs rosbag input
"""

import rospy
import rosnode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String

def pcl2_cb(msg):
    pointcloud2_renamed_pub.publish(msg)
    
def t265_odom_cb(msg):
    tmp_odom = Odometry()
    tmp_odom.pose.pose.position = msg.pose.position
    tmp_odom.pose.pose.orientation = msg.pose.orientation
    t265_odom_sample_renamed_pub.publish(tmp_odom)

if __name__ == "__main__":

    rospy.init_node('topic_renamer', anonymous=True)

    ## boralis_tf_broadcaster rebroadcaster
    # rebroadcast the topics
    pointcloud2_topic = "/iris1/ouster/pointcloud2"
    local_position_topic = "/uav2/mavros/local_position/pose"

    # to the following topics
    pointcloud2_topic_rebroadcast = "/uav2/os_cloud_node/points"
    t265_odom_sample_rebroadcast = "/uav2/t265/odom/sample" # nav_msgs::Odometry

    # subscribers
    pointcloud2_sub = rospy.Subscriber(pointcloud2_topic, PointCloud2, pcl2_cb)
    local_pose_sub = rospy.Subscriber(local_position_topic, PoseStamped, t265_odom_cb)

    # publishers
    pointcloud2_renamed_pub = rospy.Publisher(pointcloud2_topic_rebroadcast, PointCloud2, queue_size=10)
    t265_odom_sample_renamed_pub = rospy.Publisher(t265_odom_sample_rebroadcast, Odometry, queue_size=10)

    rospy.loginfo("Topic Renamer Script Begin")
    rospy.spin()
