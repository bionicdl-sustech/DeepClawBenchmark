# Sim-2-Real

## Introduction to CoppeliaSim (V-REP) and PyRep

We use CoppeliaSim and PyRep for the simulation in deepclaw.

The robot simulator CoppeliaSim, with integrated development environment, is based on a distributed control architecture: each object/model can be individually controlled via an embedded script, a plugin, a ROS or BlueZero node, a remote API client, or a custom solution. This makes CoppeliaSim very versatile and ideal for multi-robot applications. Controllers can be written in C/C++, Python, Java, Lua, Matlab or Octave.

PyRep is a toolkit for robot learning research, built on top of CoppeliaSim (previously called V-REP).

Please refer to the deepclaw/sim2real/simulation folder for installation instructions and usage examples.

## Project 1: Kinematic Picking in PyRep
The Kinematic picking project aims to build a simple pick and place scene in CoppeliaSim and complete the pick and place task without using any vision input. The project also demonstrates how to generate a predefined path and let the Franka arm go through the waypoints.

Please refer to the deepclaw/sim2real/simulation/tasks folder for instructions.

## Project 2: Simulated Robot Player
The project aims to build a robot player of Tic-Tac-Toe in simulation.

The V-rep Scene file is provided and for making robot move and play with human, we have to implement 3 basic elements:

- Computer Vision

    Try to recognize the object on the desk, classify the category of object and calculate the real world position of object via depth image and the previous result you got. It is essential information you should know if you want your virtual robot put the chess on the correct position. 

- The decision maker(Minmax or reinforcement learning method)

    The robot need to know where to put the chess that can lead it go to win. The classical method MINMAX is recommended and the reinforcement learning method that based on MDP is happily welcomed.

- Robot control

    Control robot to run on the trajectory that lead robot gripper grasp chess and release chess stable and robust.

## Project 3: Claw Machine
The project aims to program a franka robot to claw the toys in front of the robot and put them to a box next to robot. Please use the graspNet model in DeepClaw, which is an end2end model that take an image as input and output the best position and pose to pick. Please read [our paper](https://arxiv.org/abs/2003.01582) for a detailed explanation of the grasping neural network.
