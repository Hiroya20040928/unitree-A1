#!/bin/bash
export SUDO_ASKPASS=/home/unitree/RobotVisionSystem/passwd.sh
sudo /etc/init.d/network-manager stop
sudo ifconfig wlan0 192.168.12.1 netmask 255.255.255.0
sudo /etc/init.d/dnsmasq restart
sudo hostapd -B /etc/hostapd/hostapd.conf

cd /home/unitree/RobotVisionSystem/ 
sudo ./build/RobotVisionSystem -i "./build/Src/SystemInputPlugin/vision.so -fps 30" -o "./build/Src/SystemOutputPlugin/httpd.so -w ./WWW/"
