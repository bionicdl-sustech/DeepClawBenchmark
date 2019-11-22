#!/bin/sh

echo "Installing DeepClaw in UR arm with RealSense D435 support"

# numpy, yaml, pyrealsense2, cv2
echo "realsense libraries installing..."
echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE
sudo apt-get update
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils

pip install numpy
pip install PyYaml
pip install pyrealsense2
pip install opencv-python
