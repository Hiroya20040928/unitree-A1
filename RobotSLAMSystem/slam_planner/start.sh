#!/bin/sh
##############
# Author: ian
##############
#sleep 10
NUM=$(ps -ef | grep index.js | grep -v "grep" | wc -l)
if [ $NUM -eq 0 ]; then
   rm -rf /home/$USER/RobotSLAMSystem/slam_planner/Files/*
fi
export SUDO_ASKPASS=/home/$USER/RobotSLAMSystem/slam_planner/PASSWD

IDsdk=`ps -ef | grep "lcm_server_high" | grep -v "$0" | grep -v "grep" | awk '{print $2}'`
if [ "$IDsdk" != "" ]; then
    sudo -A kill $IDsdk
fi

ID=`ps -ef | grep "slam_planner_online" | grep -v "$0" | grep -v "grep" | awk '{print $2}'`
if [ "$ID" != "" ]; then
    sudo -A kill $ID
fi

IP=192.168.11.1
loss=`ping -c 1 $IP | grep loss | awk '{print $6}' | awk -F "%" '{print $1}'` 


if [ "$loss" = "" ] || [ $loss -ne 0 ];then
   echo "NULL, ping $IP Failed !"
else
	IDsdk=`ps -ef | grep "lcm_server_high" | grep -v "$0" | grep -v "grep" | awk '{print $2}'`
	if [ "$IDsdk" != "" ]; then
    	sudo -A kill $IDsdk
    	sleep 3
	fi
	gnome-terminal -- bash -c "cd ~/unitree_legged_sdk/build;sleep 2; sudo -A ./lcm_server_high; exec bash"
	COUNT_0=$(ps -ef | grep lcm_server_high | grep -v "grep" | wc -l)
	sleep 1

	gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash; source /home/$USER/catkin_ws/devel/setup.bash; roslaunch slam_planner slam_planner_online.launch; exec bash"
	COUNT_1=$(ps -ef | grep slam_planner_online | grep -v "grep" | wc -l)

	sleep 10

	if [ $COUNT_0 -ge 1 ] && [ $COUNT_1 -ge 1 ]; then
		echo "SUCCESS"
	else
		echo "sth is error, please restart the application !"
	fi
	sleep 3

fi
