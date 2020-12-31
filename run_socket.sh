#!/bin/bash

#export ROS_MASTER_URI=http://192.168.2.109:11311
#export ROS_HOSTNAME=192.168.2.109

export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_HOSTNAME=10.42.0.1

source /opt/ros/melodic/setup.bash
cd /home/coxsys/coxsys_ui_server/files
/usr/bin/python3 flask_server_socket.py
