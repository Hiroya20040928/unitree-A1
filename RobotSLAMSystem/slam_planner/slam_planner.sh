#!/bin/sh
################
# Author: ian
################
sleep 10

NUM=$(ps -ef | grep index.js | grep -v "grep" | wc -l)
if [ $NUM -eq 0 ]; then
   echo "clear map cache ....."
   rm -rf /home/$USER/RobotSLAMSystem/slam_planner/Files/* 
fi

export SUDO_ASKPASS=/home/$USER/RobotSLAMSystem/slam_planner/PASSWD

sudo -A ifconfig lo multicast
sudo -A route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
sleep 10
IP=192.168.11.1
loss=`ping -c 1 $IP | grep loss | awk '{print $6}' | awk -F "%" '{print $1}'` 
Vision=`ps -ef | grep ./build | grep -v "$0" | grep -v "grep" | awk '{print $2}'`

echo "loss="$loss

if [ "$loss" = "" ] || [ $loss -ne 0 ];then
   echo "NULL, ping $IP Failed !"
else
	echo "ping $IP OK !"
	if [ "$Vision" != "" ];then
        	sudo -A kill $Vision
    	else
        	sleep 5
        	Vision2=`ps -ef | grep ./build | grep -v "$0" | grep -v "grep" | awk '{print $2}'`
    	fi

    	if [ "$Vision2" != "" ];then
        	sudo kill -A $Vision2
    	fi

	gnome-terminal -- bash -c "cd ~/unitree_legged_sdk/build; sudo -A ./lcm_server_high; exec bash"
	COUNT_0=$(ps -ef | grep lcm_server_high | grep -v "grep" | wc -l)

	if [ $? -eq 0 ] && [ $COUNT_0 -ge 1 ]; then
		echo "1. start the laikago_sdk .. "
	else
		echo "It doesn't find unitree_legged_sdk! "
	fi
	sleep 25

	gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash; source /home/$USER/catkin_ws/devel/setup.bash; roslaunch slam_planner slam_planner_online.launch; exec bash"
	COUNT_SLAM=$(ps -ef | grep slam_planner_online | grep -v "grep" | wc -l)

	if [ $? -eq 0 ] && [ $COUNT_SLAM -ge 1 ]; then
    		echo "2. SLAM is OK! "
	else
    		echo "It can't start slam application. "
	fi
	sleep 5

	gnome-terminal -- bash -c "source /opt/ros/melodic/setup.bash; source /home/$USER/catkin_ws/devel/setup.bash; cd /home/$USER/RobotSLAMSystem/slam_planner/; node index.js; exec bash"
	COUNT_Node=$(ps -ef | grep index.js | grep -v "grep" | wc -l)

	if [ $? -eq 0 ] && [ $COUNT_Node -ge 1 ]; then
		echo "Everything Is OK !"
	else
		echo "App communication is failed!"	
	fi
fi
