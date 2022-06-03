#!/usr/bin/env python

import functools
import os
import subprocess

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

def ns_join(*names):
  return functools.reduce(rospy.names.ns_join, names, "")

class Node:
  def __init__(self, name, pid):
    self.name = name
    self.proc = psutil.Process(pid)
    self.cpu_publisher = rospy.Publisher(ns_join("~", name[1:], "cpu"), Float32, queue_size=20)
    self.mem_publisher = rospy.Publisher(ns_join("~", name[1:], "mem"), UInt64, queue_size=20)

  def publish(self):
    self.cpu_publisher.publish(Float32(self.proc.cpu_percent()))
    self.mem_publisher.publish(UInt64(self.proc.memory_info().rss))

  def alive(self):
    return self.proc.is_running()


class RecordNode:
    def __init__(self, name, pid, logs_path):
        self._name = name[1:] # strip "/" infront of the name
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
        now = datetime.now()
        cpu_usage = str(Float32(self.proc.cpu_percent()))
        mem_usage = str(UInt64(self.proc.memory_info().rss))

        f = open(self.cpu_log_file, 'a')
        f.write("{},{}\n".format(cpu_usage, now))
        f.close()

        f = open(self.mem_log_file, 'a')
        f.write("{},{}\n".format(mem_usage, now))
        f.close()

    def alive(self):
        return self.proc.is_running()


if __name__ == "__main__":
  rospy.init_node("local_ros_node_logger")
  master = rospy.get_master()

  poll_period = 0.5

  this_ip = os.environ.get("ROS_IP")
  node_map = {}
  ignored_nodes = set()

  now = datetime.now().strftime("%d_%m_%Y_time:%H_%M_%S")
  path_to_store_logs = os.path.expanduser('~/Diagnosis/NodesLogs/') + now + "/"
  os.mkdir(path_to_store_logs)

  while not rospy.is_shutdown():
    for node in rosnode.get_node_names():
      if node in node_map or node in ignored_nodes:
        continue

      node_api = rosnode.get_api_uri(master, node)[2]
      if not node_api:
        rospy.logerr("[node cpu logger] failed to get api of node %s (%s)" % (node, node_api))
        continue

      ros_ip = node_api[7:] # strip http://
      ros_ip = ros_ip.split(':')[0] # strip :<port>/
      local_node = "localhost" in node_api or \
                   "127.0.0.1" in node_api or \
                   (this_ip is not None and this_ip == ros_ip) or \
                   subprocess.check_output("hostname").decode('utf-8').strip() in node_api
      if not local_node:
        ignored_nodes.add(node)
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
          node_map[node] = RecordNode(name=node, pid=pid, logs_path=path_to_store_logs)
          rospy.loginfo("[node cpu logger] adding new node %s" % node)

    for node_name, node in list(node_map.items()):
      if node.alive():
        node.record()
      else:
        rospy.logwarn("[node cpu logger] lost node %s" % node_name)
        del node_map[node_name]

    rospy.sleep(poll_period)