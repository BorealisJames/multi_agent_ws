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
    # tmp = PointCloud2
    # tmp = msg
    rospy.loginfo("Topic Renamer Script Begin")
    print(type(msg.data()))
    #pointcloud2_renamed_pub.publish(msg.data)
    
def t265_odom_cb(msg):
    #t265_odom_sample_renamed_pub.publish(msg.data)
    pass

if __name__ == "__main__":

    rospy.init_node('topic_renamer', anonymous=True)

    ## boralis_tf_broadcaster rebroadcaster
    # rebroadcast the topics
    pointcloud2_topic = "/iris0/ouster/pointcloud2"
    t265_odom_sample_topic = "/uav1/mavros/local_position/odom"

    # to the following topics
    pointcloud2_topic_rebroadcast = "/os_cloud_nodewtf/points"
    t265_odom_sample_rebroadcast = "/uav1/t265/odom/sample" # nav_msgs::Odometry

    # subscribers
    pointcloud2_sub = rospy.Subscriber(pointcloud2_topic, PointCloud2, pcl2_cb)
    t265_odom_sample_sub = rospy.Subscriber(t265_odom_sample_topic, Odometry, t265_odom_cb)

    # publishers
    pointcloud2_renamed_pub = rospy.Publisher(pointcloud2_topic_rebroadcast, PointCloud2, queue_size=10)
    t265_odom_sample_renamed_pub = rospy.Publisher(t265_odom_sample_rebroadcast, Odometry, queue_size=10)

    rospy.loginfo("Topic Renamer Script Begin")
    rospy.spin()
