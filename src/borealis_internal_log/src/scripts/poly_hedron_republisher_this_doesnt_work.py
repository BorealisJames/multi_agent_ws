#!/usr/bin/env python

from numpy import poly
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from decomp_ros_msgs.msg import PolyhedronArray


class talker:

    def __init__(self):
        self.poly = PolyhedronArray()

        pub = rospy.Publisher('Polyhedrone_Republisher', PolyhedronArray, queue_size=10)
        subscriber = rospy.Subscriber('/uav1/borealis_teaming_planner/polyhedron_array_uav', PolyhedronArray, self.cb)

    def cb(self,msg):
        self.poly = msg
        print(self.poly)


if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    # Create a rate
    talker_= talker()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        talker_.publish_temperature()
        rate.sleep()