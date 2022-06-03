#!/usr/bin/env python2

import subprocess
from datetime import datetime
import rospy
import os
from geometry_msgs.msg import PoseStamped 
import numpy as np
import math

def localpose_cb(msg):
    global localpose, localpose_flag, np_local_pose
    localpose.pose.position.x = msg.pose.position.x
    localpose.pose.position.y = msg.pose.position.y
    localpose.pose.position.z = msg.pose.position.z

    np_local_pose[0] = msg.pose.position.x
    np_local_pose[1] = msg.pose.position.y
    np_local_pose[2] = msg.pose.position.z

def assignedpose_cb(msg):
    global assignedpose, assignedpose_flag, np_assigned_pose
    assignedpose.pose.position.x = msg.pose.position.x
    assignedpose.pose.position.y = msg.pose.position.y
    assignedpose.pose.position.z = msg.pose.position.z
    assignedpose_flag = True

    np_assigned_pose[0] = msg.pose.position.x
    np_assigned_pose[1] = msg.pose.position.y
    np_assigned_pose[2] = msg.pose.position.z


def republisher_cb():
    global assignedpose, np_local_pose, np_assigned_pose
    unit_vector_1 = np_local_pose / np.linalg.norm(np_local_pose)
    unit_vector_2 = np_assigned_pose / np.linalg.norm(np_assigned_pose)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)

    pi = math.pi
    # greater than 180
    if angle > pi/2:
        pass
    else:
        filtered_pub.publish(assignedpose)


if __name__ == "__main__":
    rospy.init_node('borealis_mavros_log')
    rospy.loginfo("Borealis internal log running")
    localpose = PoseStamped()
    assignedpose = PoseStamped()
    filtered_pose = PoseStamped()


    localpose_flag = False
    assignedpose_flag = False
    drone_number = os.getenv('DRONE_NUMBER')

    localposition_topic = "/uav" + str(drone_number) + "/mavros/local_position/pose"
    assignedpose_topic = "/uav" + str(drone_number) + "/teaming_planner/assigned_virtual_position"
    filtered_topic = "/uav" + str(drone_number) + "/teaming_planner/assigned_virtual_position_filtered"

    # Sub
    localposition_sub = rospy.Subscriber(localposition_topic, PoseStamped, localpose_cb)
    assignedpose_sub = rospy.Subscriber(assignedpose_topic, PoseStamped, assignedpose_cb)
    # Pub
    filtered_pub = rospy.Publisher(filtered_topic,PoseStamped, queue_size=10)


    np_local_pose = np.array([0,0,0])  
    np_assigned_pose = np.array([0,0,0])  
    #print the result    
    print(np.cross(a,b))

    timer = rospy.Timer(1, republisher_cb)
    timer.start()

    rospy.spin()