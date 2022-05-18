#!/usr/bin/env python3.7

from ast import arg
import multiprocessing
import signal
import time
import os
import subprocess
from datetime import datetime
from pythonping import ping
import psutil

def handler(signum, frame):
    print("Stopping logger")
    p1.kill()
    p2.kill()
    p3.kill()
    p4.kill()
    exit()

def write_ping(ip_add, ping_file):
    output = ping(ip_add, verbose=False)
    now = datetime.now().strftime("%d_%m_%Y_time_%H_%M_%S.%f")[:-3] 
    f = open(ping_file, 'a')
    f.write(str(output))
    f.write("\n")
    f.write(now)
    f.write(",")
    f.write("\n")
    f.write("\n")
    f.close()

def ping_timer(hz):
    global drone_number, path_to_store_logs

    if drone_number == "1":
        ping_file = path_to_store_logs + '/ping_uav1_to_uav2.txt'
    if drone_number == "2":
        ping_file = path_to_store_logs + '/ping_uav2_to_uav1.txt'

    f = open(ping_file, 'w')
    f.write("Starting Ping Logs")
    f.close()

    while True:
        time.sleep(hz)
        if drone_number == "1":
            write_ping('192.168.1.62',ping_file)
        if drone_number == "2":
            write_ping('192.168.1.63',ping_file)

def read_wifi_strength():
    p = subprocess.Popen("iwconfig", stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out = p.stdout.read().decode()
    p.communicate()

    return out

def write_wifi_strength(wifi_file):
    output = read_wifi_strength()
    now = datetime.now().strftime("%d_%m_%Y_time_%H_%M_%S.%f")[:-3] 
    f = open(wifi_file, 'a')
    f.write(str(output))
    f.write(now)
    f.write(",")
    f.write("\n")
    f.close()

def wifi_timer(hz):
    global path_to_store_logs

    wifi_file = path_to_store_logs + "/wifi_signal_strength_log.txt"
    while True:
        time.sleep(hz)
        write_wifi_strength(wifi_file)

def write_cpu_strength(cpu_file, hz):
    output = psutil.cpu_percent(hz) # avg cpu usage for 0.5 seconds
    now = datetime.now().strftime("%d_%m_%Y_time_%H_%M_%S.%f")[:-3] 
    f = open(cpu_file, 'a')
    f.write(str(output))
    f.write("\n")
    f.write(now)
    f.write(",")
    f.write("\n")
    f.close()

def cpu_timer(hz):
    global path_to_store_logs

    cpu_file = path_to_store_logs + "/cpu_strength_log.txt"
    while True:
        write_cpu_strength(cpu_file, hz)

def write_ram_usage(ram_file):
    output = psutil.virtual_memory()[2] # ram usage 
    now = datetime.now().strftime("%d_%m_%Y_time_%H_%M_%S.%f")[:-3] 
    f = open(ram_file, 'a')
    f.write(str(output))
    f.write("\n")
    f.write(now)
    f.write(",")
    f.write("\n")
    f.close()

def ram_timer(hz):
    global path_to_store_logs

    ram_file = path_to_store_logs + "/ram_usage.txt"
    while True:
        time.sleep(0.5)
        write_ram_usage(ram_file)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, handler)
    drone_number = os.getenv('DRONE_NUMBER')
    print("Drone number is : {}".format(drone_number))

    p1 = multiprocessing.Process(target=wifi_timer,args=[0.5])
    p2 = multiprocessing.Process(target=ping_timer,args=[0.5])
    p3 = multiprocessing.Process(target=cpu_timer,args=[0.5]) # in percantage
    p4 = multiprocessing.Process(target=ram_timer,args=[0.5]) # in percantage

    now = datetime.now().strftime("%d_%m_%Y_time_%H_%M_%S")
    path_to_store_logs = os.path.expanduser('~/Diagnosis/') + now
    # create the directory according to time stamp

    os.mkdir(path_to_store_logs)
    f = open(os.path.expanduser('~/Diagnosis/') + "last_run.txt"  , 'w')
    f.write(now)
    f.close()

    p1.start()
    p2.start()
    p3.start()
    p4.start()
