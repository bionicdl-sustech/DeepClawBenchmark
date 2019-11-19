# Copyright (c) 2019 by Hank. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8


class ArmController(object):
    def __init__(self):
        pass

    def move_j(self, joint, velocity, accelerate, solution_space):
        raise NotImplementedError(' move joint function does not implement. ')

    def move_p(self, position, velocity, accelerate, solution_space):
        raise NotImplementedError(' move position function does not implement. ')

    def get_state(self):
        raise NotImplementedError(' get status function does not implement. ')

    def verify_state(self, variable_name, target_value, error):
        raise NotImplementedError(' check status function does not implement. ')
