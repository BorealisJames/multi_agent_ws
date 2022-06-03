#!/usr/bin/env python2

import subprocess
from datetime import datetime
import rospy
import os
from geometry_msgs.msg import PoseStamped 

def localpose_cb(msg):
    global localpose, localpose_flag
    localpose.pose.position.x = msg.pose.position.x
    localpose.pose.position.y = msg.pose.position.y
    localpose.pose.position.z = msg.pose.position.z
    localpose_flag = True

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
        print("Recording mavros local position {}, {}, {}".format(x,y,z))
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
        print("Recording mavros setpoint position {}, {}, {}".format(x,y,z))
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
        print("Recording assignedd pose position {}, {}, {}".format(x,y,z))
        f.close()


if __name__ == "__main__":
    rospy.init_node('borealis_mavros_log')
    rospy.loginfo("Borealis internal log running")
    localpose = PoseStamped()
    setpointpose = PoseStamped()
    assignedpose = PoseStamped()

    localpose_flag = False
    setpoint_flag = False
    assignedpose_flag = False

    loop_rate = 0.5 

    drone_number = os.getenv('DRONE_NUMBER')
    now = datetime.now().strftime("%d_%m_%Y_time:%H_%M_%S")
    path_to_store_logs = os.path.expanduser('~/Diagnosis/MavrosLogs/') + now
    os.mkdir(path_to_store_logs)

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

    localposition_topic = "/uav" + str(drone_number) + "/mavros/local_position/pose"
    setpoint_topic = "/uav" + str(drone_number) + "/mavros/setpoint_position/local"
    assignedpose_topic = "/uav" + str(drone_number) + "/teaming_planner/assigned_virtual_position"

    localposition_sub = rospy.Subscriber(localposition_topic, PoseStamped, localpose_cb)
    setpoint_sub = rospy.Subscriber(setpoint_topic, PoseStamped, setpoint_cb)
    assignedpose_sub = rospy.Subscriber(assignedpose_topic, PoseStamped, assignedpose_cb)

    setpoint_timer = rospy.Timer(rospy.Duration(loop_rate), record_localpose)
    localpose_timer = rospy.Timer(rospy.Duration(loop_rate), record_setpointpose)
    localpose_timer = rospy.Timer(rospy.Duration(loop_rate), record_assignedpose)
    print("Mavros logs")
    rospy.spin()