# Copyright (c) 2019 by Hank. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8


class Task(object):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        self.perception_system = perception_system
        self.manipulation_system = manipulation_system
        self.is_debug = is_debug

    def task_display(self):
        raise NotImplementedError(' goHome method does not implement. ')

    def subtask_display(self):
        raise NotImplementedError(' goHome method does not implement. ')



