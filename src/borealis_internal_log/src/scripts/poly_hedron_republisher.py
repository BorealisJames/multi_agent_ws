#!/usr/bin/env python

from numpy import poly
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from decomp_ros_msgs.msg import PolyhedronArray


class talker:

    def __init__(self):
        self.poly = PolyhedronArray()

        self.pub = rospy.Publisher('Polyhedrone_Republisher', PolyhedronArray, queue_size=10)
        self.subscriber = rospy.Subscriber('/uav1/borealis_teaming_planner/polyhedron_array_uav', PolyhedronArray, self.cb)

    def cb(self,msg):
        self.poly = msg

    def publish_data(self, event=None):
        self.pub.publish(self.poly)


if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    # Create a rate
    talker_= talker()
    rate = rospy.Rate(1)

    # Create another ROS Timer for publishing data
    timer = rospy.Timer(rospy.Duration(1.0/10.0), talker_.publish_data)
    # Don't forget this or else the program will exit
    rospy.spin()

    while not rospy.is_shutdown():
        talker_.pub.publish(talker_.poly)
        rate.sleep()