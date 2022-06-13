#!/usr/bin/env python

import functools
import os
import subprocess

from click import argument

import rosnode
import rospy

import psutil

try:
  from xmlrpc.client import ServerProxy
except ImportError:
  from xmlrpclib import ServerProxy

from std_msgs.msg import Float32, UInt64
from datetime import datetime

# adapted from https://github.com/alspitz/cpu_monitor 
# additional reference -> https://github.com/ros-visualization rqt, git hub where i reference from rqt system_plot

def ns_join(*names):
  return functools.reduce(rospy.names.ns_join, names, "")

class RecordNode:
    def __init__(self, name, pid, logs_path):
        self._name = name[1:] # strip "/" infront of the name
        
        self._name = str(self._name.replace("/", "_"))

        self._pid = pid
        path_to_store_node_logs = logs_path + self._name + "/"
        os.mkdir(path_to_store_node_logs)

        self.cpu_log_file = path_to_store_node_logs + self._name + "_node_cpu_usage.csv" 
        self.mem_log_file = path_to_store_node_logs + self._name + "_node_mem_usage.csv" 
        self.proc = psutil.Process(pid)

        f = open(self.cpu_log_file, 'w')
        f.write("% Usage, time(24h)\n")
        f.close()

        f = open(self.mem_log_file, 'w')
        f.write("% Usage, time(24h)\n")
        f.close()

    def record(self):
        self.now = datetime.now()
        cpu_usage = str(Float32(self.proc.cpu_percent()))
        mem_usage = str(UInt64(self.proc.memory_percent()))
        
        cpu_usage = cpu_usage[5:] # strip the "Data: " keyword
        mem_usage = mem_usage[5:] # strip the "Data: " keyword
        f = open(self.cpu_log_file, 'a')
        f.write("{},{}\n".format(cpu_usage, self.now))
        f.close()

        f = open(self.mem_log_file, 'a')
        f.write("{},{}\n".format(mem_usage, self.now))
        f.close()

    def alive(self):
        return self.proc.is_running()

class mainROS:
  def __init__(self):
    rospy.init_node("local_ros_node_logger")
    self.master = rospy.get_master()

    self.fast_period = 5
    self.slow_period = 6

    self.this_ip = os.environ.get("ROS_IP")
    self.node_map = {}
    self.ignored_nodes = set()

    self.now = datetime.now().strftime("%d_%m_%Y_time:%H_%M_%S")
    self.path_to_store_logs = os.path.expanduser('~/Diagnosis/NodesLogs/') + self.now + "/"
    os.mkdir(self.path_to_store_logs)

    f = open(os.path.expanduser('~/Diagnosis/MavrosLogs/') + "last_run.txt"  , 'w')
    f.write(self.now)
    f.close()

    self.update_nodes_dct_timer = rospy.Timer(rospy.Duration(self.slow_period), self.update_node_map)
    self.record_nodes_stats_timer = rospy.Timer(rospy.Duration(self.fast_period), self.record_nodes_stats)

    rospy.spin()

  def update_node_map(self, timer):
    for node in rosnode.get_node_names():
      if node in self.node_map or node in self.ignored_nodes:
        continue

      node_api = rosnode.get_api_uri(self.master, node)[2]
      if not node_api:
        rospy.logerr("[node cpu logger] failed to get api of node %s (%s)" % (node, node_api))
        continue

      ros_ip = node_api[7:] # strip http://
      ros_ip = ros_ip.split(':')[0] # strip :<port>/
      local_node = "localhost" in node_api or \
                  "127.0.0.1" in node_api or \
                  (self.this_ip is not None and self.this_ip == ros_ip) or \
                  subprocess.check_output("hostname").decode('utf-8').strip() in node_api
      if not local_node:
        self.ignored_nodes.add(node)
        rospy.loginfo("[node cpu logger] ignoring node %s with URI %s" % (node, node_api))
        continue
      try:
        resp = ServerProxy(node_api).getPid('/NODEINFO')
      except:
        rospy.logerr("[node cpu logger] failed to get pid of node %s (api is %s)" % (node, node_api))
      else:
        try:
          pid = resp[2]
        except:
          rospy.logerr("[node cpu logger] failed to get pid for node %s from NODEINFO response: %s" % (node, resp))
        else:
          self.node_map[node] = RecordNode(name=node, pid=pid, logs_path=self.path_to_store_logs)
          rospy.loginfo("[node cpu logger] adding new node %s" % node)

  def record_nodes_stats(self, timer):
    for node_name, node in list(self.node_map.items()):
      if node.alive():
        node.record()
      else:
        rospy.logwarn("[node cpu logger] lost node %s" % node_name)
        del self.node_map[node_name]

if __name__ == "__main__":
  main_ros = mainROS()
