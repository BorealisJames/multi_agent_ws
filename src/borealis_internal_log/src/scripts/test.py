from datetime import datetime
from re import I
import rospy
import os
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import math
from tf.transformations import *
import numpy as np
import math

# rotation 

# Relative rotation q_r from q_1 to q_2
# q_2 = q_r*q_1
# therefore q_r = q_2*q_1_inverse
def pose_diff(pose_stamped_previous, pose_staped_now):
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

def pose_addition(vector_pose_stamped, pose_staped_now):
    q_rot = np.zeros(4)
    q_origin = np.zeros(4)
    new_pose_stamped = PoseStamped()

    # position vector diff 
    new_pose_stamped.pose.position.x = pose_staped_now.pose.position.x + vector_pose_stamped.pose.position.x
    new_pose_stamped.pose.position.y = pose_staped_now.pose.position.y + vector_pose_stamped.pose.position.y
    new_pose_stamped.pose.position.z = pose_staped_now.pose.position.z + vector_pose_stamped.pose.position.z

    q_origin[0] = pose_staped_now.pose.orientation.x
    q_origin[1] = pose_staped_now.pose.orientation.y
    q_origin[2] = pose_staped_now.pose.orientation.z
    q_origin[3] = pose_staped_now.pose.orientation.w

    q_rot[0] = vector_pose_stamped.pose.orientation.x
    q_rot[1] = vector_pose_stamped.pose.orientation.y
    q_rot[2] = vector_pose_stamped.pose.orientation.z
    q_rot[3] = vector_pose_stamped.pose.orientation.w

    q_new = quaternion_multiply(q_rot, q_origin)

    new_pose_stamped.pose.orientation.x = q_new[0]
    new_pose_stamped.pose.orientation.y = q_new[1]
    new_pose_stamped.pose.orientation.z = q_new[2]
    new_pose_stamped.pose.orientation.w = q_new[3]


    return new_pose_stamped


if __name__ == "__main__":
    # Take goal in UWB frame - Pose in UWB drone frame
    # Take this vector
    # Add this vector to the goal in t265 frame

    rospy.init_node("kekw")
    # Method 1 manual method 
    t265_pose = PoseStamped()
    goal_in_t265_pose  = PoseStamped()
    uwb_pose = PoseStamped()
    goal_in_uwb_pose = PoseStamped()
    vector_diff = PoseStamped()
    final_pose = PoseStamped()

    # drone pose in t265
    t265_pose.pose.position.x = 0.1
    t265_pose.pose.position.y = 0.1
    t265_pose.pose.position.z = 0.1

    q_t265 = quaternion_from_euler(math.pi/2, 0, 0, axes='sxyz')
    (roll, pitch, yaw) = euler_from_quaternion(q_t265, axes='sxyz')

    t265_pose.pose.orientation.x = q_t265[0]
    t265_pose.pose.orientation.y = q_t265[1]
    t265_pose.pose.orientation.z = q_t265[2]
    t265_pose.pose.orientation.w = q_t265[3]

    # drone pose in uwb
    uwb_pose.pose.position.x = 1.1
    uwb_pose.pose.position.y = 1.1
    uwb_pose.pose.position.z = 1.1

    q_uwb = quaternion_from_euler(0, 0, 0, axes='sxyz')
    uwb_pose.pose.orientation.x = q_uwb[0]
    uwb_pose.pose.orientation.y = q_uwb[1]
    uwb_pose.pose.orientation.z = q_uwb[2]
    uwb_pose.pose.orientation.w = q_uwb[3]

    # goal in uwb pose
    goal_in_uwb_pose.pose.position.x = 10
    goal_in_uwb_pose.pose.position.y = 10
    goal_in_uwb_pose.pose.position.z = 10

    q_goal = quaternion_from_euler(0, 0, 0, axes='sxyz')
    goal_in_uwb_pose.pose.orientation.x = q_goal[0]
    goal_in_uwb_pose.pose.orientation.y = q_goal[1]
    goal_in_uwb_pose.pose.orientation.z = q_goal[2]
    goal_in_uwb_pose.pose.orientation.w = q_goal[3]

    vector_diff = pose_diff(uwb_pose, goal_in_uwb_pose)

    print("vector_diff diff x is", vector_diff.pose.position.x)
    print("vector_diff diff y is ", vector_diff.pose.position.y)
    print("vector_diff diff z is ", vector_diff.pose.position.z)

    print("vector_diff diff x_rot ", vector_diff.pose.orientation.x)
    print("vector_diff diff y_rot ", vector_diff.pose.orientation.y)
    print("vector_diff diff z_rot", vector_diff.pose.orientation.z)
    print("vector_diff diff w_rot", vector_diff.pose.orientation.w)

    final_pose = pose_addition(vector_diff, t265_pose)
    print("final_pose diff x is", final_pose.pose.position.x)
    print("final_pose diff y is ", final_pose.pose.position.y)
    print("final_pose diff z is ", final_pose.pose.position.z)

    euler = [final_pose.pose.orientation.x, final_pose.pose.orientation.y, final_pose.pose.orientation.z, final_pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(euler, axes='sxyz')

    print("final_pose x_rot ", roll)
    print("final_pose y_rot ", pitch)
    print("final_pose z_rot", yaw)
    rospy.spin()