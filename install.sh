#!/bin/sh

# read cmd line inputs
VERSION=$1
ROBOT=$2

echo "Installing DeepClaw in ${ROBOT} arm with ${VERSION} support"

sudo apt-get install python-pip
sudo apt-get install python-opencv
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
sudo pip install pyrealsense2

# set cpu/gpu conditional libraries
case "${VERSION}"
in
cpu)
  sudo pip install tensorflow
  ;;
gpu)
  sudo pip install tensorflow-gpu
  ;;
*)
  echo "Usage: $0 {cpu|gpu} {ur|franka|aubo|denso}"
  exit 1
esac

case "${ROBOT}"
in
ur)
  sudo pip install urx
  ;;
franka)
  sudo apt install ros-kinetic-libfranka ros-kinetic-franka-ros
  ;;
denso)
  cd ..
  git clone https://github.com/DENSORobot/denso_cobotta_ros.git
  cd DeepClawBenchmark
  ;;
*)
  echo "Usage: $0 {cpu|gpu} {ur|franka|aubo|denso}"
  exit 1
esac

