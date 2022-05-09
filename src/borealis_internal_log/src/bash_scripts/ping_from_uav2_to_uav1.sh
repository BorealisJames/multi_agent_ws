#!/bin/bash
diagnostic_directory=~/Diagnosis;
path_file_name=/uav2_to_uav1_ping_$(date "+%F-%T").txt;
lol=$diagnostic_directory$path_file_name
echo "Saving file to"
echo $lol
touch $lol
ping -D 192.168.1.62 >> "$lol";
