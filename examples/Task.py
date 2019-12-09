# Copyright (c) 2019 by Hank. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8


class Task(object):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        self.arm = manipulation_system["Arm"]
        self.gripper = manipulation_system["Gripper"]
        self.camera = perception_system["Camera"]
        self.recorder = perception_system["Recorder"]
        self.is_debug = is_debug

    def task_display(self):
        raise NotImplementedError(' goHome method does not implement. ')




