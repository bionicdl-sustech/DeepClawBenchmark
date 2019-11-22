#!/bin/sh

# read cmd line inputs
CAMERA=$1
ROBOT=$2

echo "Installing DeepClaw in ${ROBOT} arm with ${CAMERA} support"

# numpy, yaml, pyrealsense2, cv2

sudo apt-get install python-pip
sudo apt-get install python-opencv
pip install --upgrade pip

# set camera libraries
case "${CAMERA}"
in
realsense)
  echo "realsense libraries installing..."
  echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' | sudo tee /etc/apt/sources.list.d/realsense-public.list
  sudo apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE
  sudo apt-get update
  sudo apt-get install librealsense2-dkms
  sudo apt-get install librealsense2-utils
  pip install pyrealsense2
  ;;
*)
  echo "Usage: $0 {cpu|gpu} ur"
  exit 1
esac

case "${ROBOT}"
in
ur)
  echo "UR robot arm requirments installing..."
  pip install urx
  ;;
*)
  echo "Usage: $0 {cpu|gpu} ur"
  exit 1
esac

