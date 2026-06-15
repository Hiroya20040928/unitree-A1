#!/usr/bin/env bash

set -euo pipefail

echo "Adding Intel RealSense apt repository for Ubuntu 18.04..."
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository -y "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main"

echo "Installing RealSense DKMS and SDK packages..."
sudo apt-get update
sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev

echo "Driver state after installation:"
modinfo uvcvideo | grep -i version || true

echo
echo "Reboot is required."
