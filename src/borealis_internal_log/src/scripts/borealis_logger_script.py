#!/usr/bin/env python3.6

import multiprocessing
import signal
import time
import os
import subprocess
from datetime import datetime
from pythonping import ping
import psutil
import re

# Borealis logger that record system performance while on flight

def handler(signum, frame):
    print("Stopping logger")
    p1.kill()
    p2.kill()
    p3.kill()
    p4.kill()
    exit()

def write_ping(ip_add, ping_file):
    out = ping(ip_add, verbose=False, count=1, size=36, timeout=8)
    now = datetime.now()
    f = open(ping_file, 'a')

    time_out_flag = "Request timed out"
    if re.findall(str(out), time_out_flag):
        print("Ping time out!")
        f.write("Timeout,")
        f.write(now)
        f.write("\n")
        print("Ping Timeout!")
    else:
        out = str(out).split('/') # ms
        avg = out[3]
        f.write(str(avg))
        f.write(",")
        f.write(str(now))
        f.write("\n")
        f.close()
        print("Ping is {}".format(str(avg)))

def ping_timer(hz):
    global drone_number, path_to_store_logs

    if drone_number == "1":
        ping_file = path_to_store_logs + '/ping_uav1_to_uav2_log.csv'
    if drone_number == "2":
        ping_file = path_to_store_logs + '/ping_uav2_to_uav1_log.csv'
    f = open(ping_file, 'a')
    f.write("Ping ms,")
    f.write("Date Time")
    f.write("\n")
    f.close()

    while True:
        time.sleep(hz)
        if drone_number == "1":
            write_ping('192.168.1.63',ping_file)
        if drone_number == "2":
            write_ping('192.168.1.62',ping_file)

def read_wifi_strength():
    p = subprocess.Popen("iwconfig", stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out = p.stdout.read().decode()
    p.communicate()

    return out

def write_wifi_strength(wifi_file):
    out = read_wifi_strength()
    # Do some string splitting
    out = out.split() 
    bit_rate = out[11] # Rate=866.7 units are in Mb/s
    Tx_power = out[13] # Tx-Power=22 units are in dbm
    signal_level = out[29] # level=-51 units are in dBm
    bit_rate = bit_rate.split('=')[1] # take splitted string after 
    Tx_power = Tx_power.split('=')[1]
    signal_level = signal_level.split('=')[1]

    now = datetime.now()
    f = open(wifi_file, 'a')
    f.write(bit_rate)
    f.write(",")
    f.write(Tx_power)
    f.write(",")
    f.write(signal_level)
    f.write(",")
    f.write(str(now))
    f.write("\n")
    f.close()
    print("Wifi bit rate is  {} Mb/s".format(str(bit_rate)))

def wifi_timer(hz):
    global path_to_store_logs

    wifi_file = path_to_store_logs + "/wifi_signal_strength_log.csv"
    f = open(wifi_file, 'a')
    f.write("bit rate Mb/s,")
    f.write("TX power dBm,")
    f.write("Signal Power dBm,")
    f.write("Date Time")
    f.write("\n")
    f.close()

    while True:
        time.sleep(hz)
        write_wifi_strength(wifi_file)

def write_cpu_strength(cpu_file, hz):
    output = psutil.cpu_percent(hz) # avg cpu usage for 0.5 seconds
    now = datetime.now()
    f = open(cpu_file, 'a')
    f.write(str(output))
    f.write(",")
    f.write(str(now))
    f.write("\n")
    f.close()
    print("CPU usage is {} %".format(str(output)))

def cpu_timer(hz):
    global path_to_store_logs

    cpu_file = path_to_store_logs + "/cpu_strength_log.csv"
    f = open(cpu_file, 'a')
    f.write("CPU usage (%),time(s)\n")
    f.close()
    while True:
        write_cpu_strength(cpu_file, hz)

def write_ram_usage(ram_file):
    output = psutil.virtual_memory()[2] # ram usage 
    now = datetime.now()
    f = open(ram_file, 'a')
    f.write(str(output))
    f.write(",")
    f.write(str(now))
    f.write("\n")
    f.close()
    print("RAM usage is {} %".format(str(output)))


def ram_timer(hz):
    global path_to_store_logs

    ram_file = path_to_store_logs + "/ram_usage_log.csv"
    f = open(ram_file, 'a')
    f.write("RAM usage (%),time(s)\n")
    f.close()

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

    now = datetime.now().strftime("%d_%m_%Y_time:%H_%M_%S")
    path_to_store_logs = os.path.expanduser('~/Diagnosis/Logs/') + now
    # create the directory according to time stamp

    os.mkdir(path_to_store_logs)
    f = open(os.path.expanduser('~/Diagnosis/Logs/') + "last_run.txt"  , 'w')
    f.write(now)
    f.close()

    p1.start()
    p2.start()
    p3.start()
    p4.start()
