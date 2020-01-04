#!/bin/sh
echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE
sudo apt-get update
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
#Developers shall install additional packages:
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
#Verify that the kernel is updated
modinfo uvcvideo | grep "version:"
#
sudo apt install python-pip
sudo pip install pyrealsense2
