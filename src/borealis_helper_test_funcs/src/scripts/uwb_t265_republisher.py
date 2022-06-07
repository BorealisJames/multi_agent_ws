#!/usr/bin/env python

## Publishes a topic with the the with/after tf transforms
  
import os
import rospy
import math
import os
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf
from tf.transformations import *
import numpy as np
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int32, Float64, String

class Republish():

    def __init__(self):
        
        self.t265_setpoint = PoseStamped()
        self.uwb_pose = PoseStamped()

        self.bool_wait = True
        # setpoint subscriber
        uav_setpoint_topic = '/uwb_uav_setpoint'
        rospy.Subscriber(uav_setpoint_topic, PoseStamped, self.setpoint_callback)

        # setpoint republisher
        uav_publish_topic = "/t265_uav_setpoint"
        uav_publisher = rospy.Publisher(uav_publish_topic , PoseStamped,queue_size=1)

        self.uav_pos_uwb = PoseWithCovarianceStamped()
        rospy.Subscriber('UAVposeUWB',  PoseWithCovarianceStamped, self.uav_uwb_callback)

        self.uav1_go_there_yaw = Float64()
        rospy.Subscriber('/uav1/command/yaw',  Float64, self.uav1_go_there_yaw_callback)
        self.uav2_go_there_yaw = Float64()
        rospy.Subscriber('/uav2/command/yaw',  Float64, self.uav2_go_there_yaw_callback)
        self.uav1_yaw_publisher = rospy.Publisher("/uav1/command/yaw/out" , Float64,queue_size=1)
        self.uav2_yaw_publisher = rospy.Publisher("/uav2/command/yaw/out" , Float64,queue_size=1)
        rate = rospy.Rate(20.0)

        while not rospy.is_shutdown():
            if self.bool_wait:
                pass
            else:
                uav_publisher.publish(self.t265_setpoint)

            print("Yaw values 1: ", self.uav1_go_there_yaw)
            print("Yaw values 2: ", self.uav2_go_there_yaw)

            self.uav1_yaw_publisher.publish(self.uav1_go_there_yaw)
            self.uav2_yaw_publisher.publish(self.uav2_go_there_yaw)
            rate.sleep()

    def setpoint_callback(self,data):
        self.bool_wait = False
        vector_diff_uav2 = self.pose_diff(self.uwb_pose, data)
        final_pose_uav2 = self.pose_addition(vector_diff_uav2, self.uav2_pos)

        self.t265_setpoint = final_pose_uav2

    def uav_uwb_callback(self, data):
        self.uwb_pose.pose.position.x = data.pose.position.position.x
        self.uwb_pose.pose.position.y = data.pose.position.position.y
        self.uwb_pose.pose.position.z = data.pose.position.position.z

        self.uwb_pose.pose.orientation.x = data.pose.position.position.x
        self.uwb_pose.pose.orientation.y = data.pose.position.position.y
        self.uwb_pose.pose.orientation.z = data.pose.position.position.z
        self.uwb_pose.pose.orientation.w = data.pose.position.position.w

    def uav1_go_there_yaw_callback(self,data):
       self.uav1_go_there_yaw=data.data

    def uav2_go_there_yaw_callback(self,data):
       self.uav2_go_there_yaw=data.data
    

    # rotation
    # Relative rotation q_r from q_1 to q_2
    # q_2 = q_r*q_1
    # therefore q_r = q_2*q_1_inverse
    def pose_diff(self, pose_stamped_previous, pose_staped_now):
        q1_inv = np.zeros(4)
        q2 = np.zeros(4)
        vector_diff = PoseStamped()

        # position vector diff 
        vector_diff.pose.position.x = pose_staped_now.pose.position.x - pose_stamped_previous.pose.position.x
        vector_diff.pose.position.y = pose_staped_now.pose.position.y - pose_stamped_previous.pose.position.y
        vector_diff.pose.position.z = pose_staped_now.pose.position.z - pose_stamped_previous.pose.position.z

        q1_inv[0] = pose_stamped_previous.pose.orientation.x
        q1_inv[1] = pose_stamped_previous.pose.orientation.y
        q1_inv[2] = pose_stamped_previous.pose.orientation.z
        q1_inv[3] = -pose_stamped_previous.pose.orientation.w # Negate for inverse

        q2[0] = pose_staped_now.pose.orientation.x
        q2[1] = pose_staped_now.pose.orientation.y
        q2[2] = pose_staped_now.pose.orientation.z
        q2[3] = pose_staped_now.pose.orientation.w

        qr = quaternion_multiply(q2, q1_inv)

        vector_diff.pose.orientation.x = qr[0]
        vector_diff.pose.orientation.y = qr[1]
        vector_diff.pose.orientation.z = qr[2]
        vector_diff.pose.orientation.w = qr[3]

        return vector_diff

    def pose_addition(self, vector_pose_stamped_to_add, pose_staped_now):
        q_rot = np.zeros(4)
        q_origin = np.zeros(4)
        new_pose_stamped = PoseStamped()

        # position vector diff 
        new_pose_stamped.pose.position.x = pose_staped_now.pose.position.x + vector_pose_stamped_to_add.pose.position.x
        new_pose_stamped.pose.position.y = pose_staped_now.pose.position.y + vector_pose_stamped_to_add.pose.position.y
        new_pose_stamped.pose.position.z = pose_staped_now.pose.position.z + vector_pose_stamped_to_add.pose.position.z

        q_origin[0] = pose_staped_now.pose.orientation.x
        q_origin[1] = pose_staped_now.pose.orientation.y
        q_origin[2] = pose_staped_now.pose.orientation.z
        q_origin[3] = pose_staped_now.pose.orientation.w

        q_rot[0] = vector_pose_stamped_to_add.pose.orientation.x
        q_rot[1] = vector_pose_stamped_to_add.pose.orientation.y
        q_rot[2] = vector_pose_stamped_to_add.pose.orientation.z
        q_rot[3] = vector_pose_stamped_to_add.pose.orientation.w

        q_new = quaternion_multiply(q_rot, q_origin)

        new_pose_stamped.pose.orientation.x = q_new[0]
        new_pose_stamped.pose.orientation.y = q_new[1]
        new_pose_stamped.pose.orientation.z = q_new[2]
        new_pose_stamped.pose.orientation.w = q_new[3]

        return new_pose_stamped

if __name__ == '__main__':
    rospy.init_node('uwb_t265_repubisher')

    node = Republish()

    rospy.spin()