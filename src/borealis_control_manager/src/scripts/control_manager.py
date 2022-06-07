#!/usr/bin/env python

## Publishes a topic with the the with/after tf transforms
  
import rospy
import os
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf
from tf.transformations import *
import numpy as np
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int32, Float64, String

class transform():

    def __init__(self):

        # Variables
        self.cmd1 = PoseStamped()
        self.cmd2 = PoseStamped()
        self.uav1_ap_uwb = PoseStamped()
        self.uav2_ap_uwb = PoseStamped()
        self.uav1_pos = PoseStamped()
        self.uav2_pos = PoseStamped()
        self.uav1_pose_uwb = PoseStamped()
        self.uav2_pose_uwb = PoseStamped()
        self.mode = String()
        self.recieved_new_callback_uav1 = False
        self.recieved_new_callback_uav2 = False

        uav1_ap_uwb_pose_topic = "/uav1/teaming_planner/uwb_assigned_virtual_position"
        uav2_ap_uwb_pose_topic = "/uav2/teaming_planner/uwb_assigned_virtual_position"
        uav1_uwb_pose_topic = "UAV1PoseUWB"
        uav2_uwb_pose_topic = "UAV2PoseUWB"
        uav1_t265_pose_topic = "uav1/mavros/local_position/pose"
        uav2_t265_pose_topic = "uav2/mavros/local_position/pose"
        
        # Subscribers
        rospy.Subscriber(uav1_ap_uwb_pose_topic,  PoseStamped, self.uav1_ap_uwb_callback)
        rospy.Subscriber(uav2_ap_uwb_pose_topic,  PoseStamped, self.uav2_ap_uwb_callback)
        rospy.Subscriber(uav1_t265_pose_topic,  PoseStamped, self.uav1_callback)
        rospy.Subscriber(uav2_t265_pose_topic,  PoseStamped, self.uav2_callback)
        rospy.Subscriber(uav1_uwb_pose_topic,  PoseWithCovarianceStamped, self.uav1_callback_uwb)
        rospy.Subscriber(uav2_uwb_pose_topic,  PoseWithCovarianceStamped, self.uav2_callback_uwb)

        # these subscribers are use to deal with cases of go there to not jitter alot
        rospy.Subscriber('/uav1/command/pose',  PoseWithCovarianceStamped, self.uav1_go_there_callback)
        rospy.Subscriber('/uav2/command/pose',  PoseWithCovarianceStamped, self.uav2_go_there_callback)
        rospy.Subscriber('/hri_mode', String, self.mode_callback)

        # Publishers
        # Publish topic for planner
        uav1_publish_topic = "/uav1/teaming_planner/t265assigned_virtual_position"
        uav2_publish_topic = "/uav2/teaming_planner/t265assigned_virtual_position"

        # uav1_publish_topic = "/uav1/mavros/setpoint_position/local"
        # uav2_publish_topic = "/uav2/mavros/setpoint_position/local"
        uav1_publisher = rospy.Publisher(uav1_publish_topic , PoseStamped,queue_size=1)
        uav2_publisher = rospy.Publisher(uav2_publish_topic , PoseStamped,queue_size=1)


        rate = rospy.Rate(20)

        # Drone position in UWB

        while not rospy.is_shutdown():

            final_pose_uav1  = PoseStamped()
            final_pose_uav2 = PoseStamped()

            vector_diff_uav1 = PoseStamped()
            vector_diff_uav2 = PoseStamped()

            print("1_diff:",[self.uav1_pose_uwb.pose.position.x,self.uav1_pose_uwb.pose.position.y,self.uav1_pose_uwb.pose.position.z])
            vector_diff_uav1 = self.pose_diff(self.uav1_pose_uwb, self.uav1_ap_uwb)
            final_pose_uav1 = self.pose_addition(vector_diff_uav1, self.uav1_pos)
            self.cmd1 = final_pose_uav1

            vector_diff_uav2 = self.pose_diff(self.uav2_pose_uwb, self.uav2_ap_uwb)
            final_pose_uav2 = self.pose_addition(vector_diff_uav2, self.uav2_pos)
            self.cmd2 = final_pose_uav2
            
            print("Mode: ", self.mode)
            print("1_diff:",[vector_diff_uav1.pose.position.x,vector_diff_uav1.pose.position.y,self.cmd1.pose.position.z])
            print("2_diff:",[vector_diff_uav2.pose.position.x,vector_diff_uav2.pose.position.y,vector_diff_uav2.pose.position.z])

            print("uav1_t265:",[self.cmd1.pose.position.x,self.cmd1.pose.position.y,self.cmd1.pose.position.z])
            print("uav2_t265:",[self.cmd2.pose.position.x,self.cmd2.pose.position.y,self.cmd2.pose.position.z])
            self.cmd1.header.frame_id = '/odom'
            self.cmd2.header.frame_id = '/odom'

            if self.mode.data == "Go_There":
                if self.recieved_new_callback_uav1:
                    uav1_publisher.publish(self.cmd1)
                    self.recieved_new_callback_uav1 = False
                if self.recieved_new_callback_uav2:
                    uav2_publisher.publish(self.cmd2)
                    self.recieved_new_callback_uav2 = False
            else:
                uav1_publisher.publish(self.cmd1)
                uav2_publisher.publish(self.cmd2)

            rate.sleep()


    def uav1_ap_uwb_callback(self,data):
        self.uav1_ap_uwb=data

    def uav2_ap_uwb_callback(self,data):
        self.uav2_ap_uwb=data

    def uav1_callback(self,data):
        self.uav1_pos=data
    
    def uav2_callback(self,data):
        self.uav2_pos=data

    def uav1_callback_uwb(self,data):
        self.uav1_pose_uwb.pose=data.pose.pose
        self.uav1_pose_uwb.header=data.header
    
    def uav2_callback_uwb(self,data):
        self.uav2_pose_uwb.pose=data.pose.pose
        self.uav2_pose_uwb.header=data.header

    def uav1_go_there_callback(self,data):
        self.uav1_go_there_pos=data
        self.recieved_new_callback_uav1=True

    def uav2_go_there_callback(self,data):
        self.uav2_go_there_pos=data
        self.recieved_new_callback_uav2=True
        
    def mode_callback(self,data):
        self.mode=data

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

        # print("Pose_stamped_now {} {} {}".format(pose_staped_now.pose.position.x, pose_staped_now.pose.position.y, pose_staped_now.pose.position.z))
        # print("Pose_stamped_previous  {} {} {}".format(pose_stamped_previous.pose.position.x, pose_stamped_previous.pose.position.y, pose_stamped_previous.pose.position.z))
        # print("Vector diff {} {} {}".format(vector_diff.pose.position.x, vector_diff.pose.position.y, vector_diff.pose.position.z))

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
    rospy.init_node('Mock_Multi_Agent')

    node = transform()

    rospy.spin()
