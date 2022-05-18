#!/usr/bin/env python3.7

import multiprocessing
import signal
import time
import os
import subprocess
from datetime import datetime
from pythonping import ping

def handler(signum, frame):
    print("Stopping logger")
    # p1.kill()
    p2.kill()
    exit()

def slow_timer():
    while True:
        time.sleep(1)
        print("Slow timer")

def write_ping(ip_add, ping_file):
    output = ping(ip_add, verbose=False)
    f = open(ping_file, 'a')
    f.write(str(output))
    f.write("\n")
    now = datetime.now().strftime("%d_%m_%Y_time_%H_%M_%S") 
    f.write(now)
    f.write("\n")
    f.write("\n")
    f.close()

def read_wifi_strength():
    p = subprocess.Popen("iwconfig", stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out = p.stdout.read().decode()
    p.communicate()

    return out

def write_wifi_strength(wifi_file):
    output = read_wifi_strength()
    f = open(wifi_file, 'a')
    f.write(str(output))
    f.write(now)
    f.write("\n")
    f.close()

def ping_timer():
    global drone_number, path_to_store_logs

    if drone_number == "1":
        ping_file = path_to_store_logs + '/ping_uav1_to_uav2.txt'
    if drone_number == "2":
        ping_file = path_to_store_logs + '/ping_uav2_to_uav1.txt'

    f = open(ping_file, 'w')
    f.write("Starting Ping Logs")
    f.close()

    while True:
        time.sleep(0.5)
        if drone_number == "1":
            write_ping('192.168.1.62',ping_file)

        if drone_number == "2":
            write_ping('192.168.1.63',ping_file)

def wifi_timer():
    global path_to_store_logs

    wifi_file = path_to_store_logs + "/wifi_signal_strength_log.txt"
    f = open(wifi_file, 'w')
    f.write("Starting wifi Logs")
    f.close()

    while True:
        time.sleep(0.5)
        write_wifi_strength(wifi_file)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, handler)
    p1 = multiprocessing.Process(target=wifi_timer)
    drone_number = os.getenv('DRONE_NUMBER')
    print("Drone number is : {}".format(drone_number))
    p2 = multiprocessing.Process(target=ping_timer)

    now = datetime.now().strftime("%d_%m_%Y_time_%H_%M_%S")
    path_to_store_logs = os.path.expanduser('~/Diagnosis/') + now
    # create the directory according to time stamp

    os.mkdir(path_to_store_logs)
    f = open(os.path.expanduser('~/Diagnosis/') + "last_run.txt"  , 'w')
    f.write(now)
    f.close()

    p2.start()
    p1.start()
