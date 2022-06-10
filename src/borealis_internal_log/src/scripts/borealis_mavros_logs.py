#!/usr/bin/env python2

import subprocess
from datetime import datetime
import rospy
import os
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# aloam
# uwb 
# vision 

def localpose_cb(msg):
    global localpose, localpose_flag
    localpose.pose.position.x = msg.pose.position.x
    localpose.pose.position.y = msg.pose.position.y
    localpose.pose.position.z = msg.pose.position.z
    localpose_flag = True

def mavros_vision_cb(msg):
    global mavros_vision, mavros_vision_flag

    mavros_vision.pose.position.x = msg.pose.position.x
    mavros_vision.pose.position.y = msg.pose.position.y
    mavros_vision.pose.position.z = msg.pose.position.z
    mavros_vision_flag = True

def aloam_cb(msg):
    global aloam_odom, aloam_flag
    aloam_odom.pose.pose.position.x = msg.pose.pose.position.x
    aloam_odom.pose.pose.position.y = msg.pose.pose.position.y
    aloam_odom.pose.pose.position.z = msg.pose.pose.position.z
    aloam_flag = True

def hri_mode_cb(msg):
    global hri_mode_file
    f = open(hri_mode_file, 'a')
    now = datetime.now()
    f.write("{},{}\n".format(msg.data,now))
    f.close()

def setpoint_cb(msg):
    global setpointpose, setpoint_flag
    setpointpose.pose.position.x = msg.pose.position.x
    setpointpose.pose.position.y = msg.pose.position.y
    setpointpose.pose.position.z = msg.pose.position.z
    setpoint_flag = True

def assignedpose_cb(msg):
    global assignedpose, assignedpose_flag
    assignedpose.pose.position.x = msg.pose.position.x
    assignedpose.pose.position.y = msg.pose.position.y
    assignedpose.pose.position.z = msg.pose.position.z
    assignedpose_flag = True

def record_localpose(event):
    global localpose, localpose_file, localpose_flag

    if localpose_flag == True:
        f = open(localpose_file, 'a')
        x = str(localpose.pose.position.x)
        y = str(localpose.pose.position.y)
        z = str(localpose.pose.position.z)
        now = datetime.now()
        # print("Recording mavros local position {}, {}, {}".format(x,y,z))
        f.write("{},{},{},{}\n".format(x,y,z,now))
        f.close()

def record_setpointpose(event):
    global setpointpose, setpoint_file, setpoint_flag

    if setpoint_flag == True:
        f = open(setpoint_file, 'a')
        x = str(setpointpose.pose.position.x)
        y = str(setpointpose.pose.position.y)
        z = str(setpointpose.pose.position.z)
        now = datetime.now()
        f.write("{},{},{},{}\n".format(x,y,z,now))
        # print("Recording mavros setpoint position {}, {}, {}".format(x,y,z))
        f.close()

def record_mavros_vision_pose(event):
    global mavros_vision, mavros_vision_file, mavros_vision_flag

    if mavros_vision_flag == True:
        f = open(mavros_vision_file, 'a')
        x = str(mavros_vision.pose.position.x)
        y = str(mavros_vision.pose.position.y)
        z = str(mavros_vision.pose.position.z)
        now = datetime.now()
        # print("Recording mavros local position {}, {}, {}".format(x,y,z))
        f.write("{},{},{},{}\n".format(x,y,z,now))
        f.close()

def record_aloam(event):
    global aloam_odom, aloam_file, aloam_flag

    if aloam_flag == True:
        f = open(aloam_file, 'a')
        x = str(aloam_odom.pose.pose.position.x)
        y = str(aloam_odom.pose.pose.position.y)
        z = str(aloam_odom.pose.pose.position.z)
        now = datetime.now()
        # print("Recording mavros local position {}, {}, {}".format(x,y,z))
        f.write("{},{},{},{}\n".format(x,y,z,now))
        f.close()

def record_localpose(event):
    global localpose, localpose_file, localpose_flag

    if localpose_flag == True:
        f = open(localpose_file, 'a')
        x = str(localpose.pose.position.x)
        y = str(localpose.pose.position.y)
        z = str(localpose.pose.position.z)
        now = datetime.now()
        # print("Recording mavros local position {}, {}, {}".format(x,y,z))
        f.write("{},{},{},{}\n".format(x,y,z,now))
        f.close()


def record_assignedpose(event):
    global assignedpose, assignedpose_file, assignedpose_flag

    if assignedpose_flag == True:
        f = open(assignedpose_file, 'a')
        x = str(assignedpose.pose.position.x)
        y = str(assignedpose.pose.position.y)
        z = str(assignedpose.pose.position.z)
        now = datetime.now()
        f.write("{},{},{},{}\n".format(x,y,z,now))
        # print("Recording assignedd pose position {}, {}, {}".format(x,y,z))
        f.close()


if __name__ == "__main__":
    rospy.init_node('borealis_mavros_log')
    rospy.loginfo("Borealis internal log running")
    localpose = PoseStamped()
    setpointpose = PoseStamped()
    assignedpose = PoseStamped()

    mavros_vision = PoseStamped()
    aloam_odom = Odometry()
        
    localpose_flag = False
    setpoint_flag = False
    assignedpose_flag = False
    aloam_flag = False
    localpose_flag = False
    mavros_vision_flag = False

    loop_rate = 0.5 

    drone_number = os.getenv('DRONE_NUMBER')
    now = datetime.now().strftime("%d_%m_%Y_time:%H_%M_%S")
    path_to_store_logs = os.path.expanduser('~/Diagnosis/MavrosLogs/') + now
    os.mkdir(path_to_store_logs)

    f = open(os.path.expanduser('~/Diagnosis/MavrosLogs/') + "last_run.txt"  , 'w')
    f.write(now)
    f.close()

    localpose_file = path_to_store_logs + "/localpose_log.csv"
    f = open(localpose_file, 'a')
    f.write("X(m), Y(m), Z(m), time(24h)\n")
    f.close()

    setpoint_file = path_to_store_logs + "/setpoint_log.csv"
    f = open(setpoint_file, 'a')
    f.write("X(m), Y(m), Z(m), time(24h)\n")
    f.close()

    assignedpose_file = path_to_store_logs + "/assignedpose_log.csv"
    f = open(assignedpose_file, 'a')
    f.write("X(m), Y(m), Z(m), time(24h)\n")
    f.close()

    hri_mode_file = path_to_store_logs + "/hri_mode_log.csv"
    f = open(hri_mode_file, 'a')
    f.write("mode, time(24h)\n")
    f.close()

    aloam_file = path_to_store_logs + "/aloam_raw_log.csv"
    f = open(aloam_file, 'a')
    f.write("X(m), Y(m), Z(m), time(24h)\n")
    f.close()

    mavros_vision_file = path_to_store_logs + "/mavros_vision_log.csv"
    f = open(mavros_vision_file, 'a')
    f.write("X(m), Y(m), Z(m), time(24h)\n")
    f.close()

    localposition_topic = "/uav" + str(drone_number) + "/mavros/local_position/pose"
    setpoint_topic = "/uav" + str(drone_number) + "/mavros/setpoint_position/local"
    assignedpose_topic = "/uav" + str(drone_number) + "/teaming_planner/t265assigned_virtual_position"
    hri_mode_topic = "/hri_mode" 
    aloam_topic = "/uav" + str(drone_number) + "/aft_mapped_to_init"
    mavros_vision_file = "/uav" + str(drone_number) + "/mavros/vision_pose/pose"

    localposition_sub = rospy.Subscriber(localposition_topic, PoseStamped, localpose_cb)
    setpoint_sub = rospy.Subscriber(setpoint_topic, PoseStamped, setpoint_cb)
    assignedpose_sub = rospy.Subscriber(assignedpose_topic, PoseStamped, assignedpose_cb)
    hri_mode_sub = rospy.Subscriber(hri_mode_topic, String, hri_mode_cb)

    aloam_sub = rospy.Subscriber(aloam_topic, Odometry, aloam_cb)
    mavros_vision_sub = rospy.Subscriber(mavros_vision_file, PoseStamped, mavros_vision_cb)

    setpoint_timer = rospy.Timer(rospy.Duration(loop_rate), record_localpose)
    localpose_timer = rospy.Timer(rospy.Duration(loop_rate), record_setpointpose)
    localpose_timer = rospy.Timer(rospy.Duration(loop_rate), record_assignedpose)

    print("Mavros logs")
    rospy.spin()