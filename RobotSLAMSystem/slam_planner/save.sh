#!/bin/bash
#########################################
# The map will be saved on time, delete in 1s later. 
########################################

flag=1
dir=/home/$USER/RobotSLAMSystem/slam_planner/Files
i=0;j=1;m=10
while [ $flag -eq 1 ]
do
	if [ $i -le 100 ];
	then
		rosrun map_server map_saver -f $dir/map$i
		echo $i	
		cp $dir/map$i.yaml $dir/map.yaml
		i=`expr $i + $j`
		
	else
		echo "The map is saved enough."
		i=`expr $i - $m`
		echo "init .. i= "$i
#		sleep 1
	fi
done

