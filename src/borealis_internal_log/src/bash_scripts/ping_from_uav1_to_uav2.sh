#!/bin/bash
diagnostic_directory=~/Diagnosis;
path_file_name=/uav1_to_uav2_ping_$(date "+%F-%T").txt;
lol=$diagnostic_directory$path_file_name
echo "Saving file to"
echo $lol
touch $lol
ping -D 192.168.1.63 >> "$lol";
