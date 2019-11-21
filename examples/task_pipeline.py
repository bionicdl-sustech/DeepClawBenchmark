# Copyright (c) 2019 by Hank. All Rights Reserved.
# !/usr/bin/python3
# coding=utf-8

# hardware initialization
# robot
robot_controller = xxx.xxxx('ip')
# gripper
gripper_controller = xxx.xxxx('ip')
# Camera
camera_controller = xxx.xxxx('ip')
# Force
force_controller = xxx.xxxx('ip')

# task begin
task_execute_condition = True
while task_execute_condition:
    # get data from sensor
    frame = camera_controller.getFrame()
    # localization


    # recognition

    # graspPlanning

    # placePlanning

    # execution
