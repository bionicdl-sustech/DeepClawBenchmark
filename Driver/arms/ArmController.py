# Copyright (c) 2019 by Hank. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8
import tf


class Controller(object):
    def __init__(self):
        self.HOME_POSE = [[0, 0, 0], [0, 0, 0]]
        self.PICK_Z = 0
        self.PLACE_Z = 0

    def goHome(self):
        raise NotImplementedError(' goHome method does not implement. ')

    def execute(self, group, plan):
        raise NotImplementedError(' execute method does not implement. ')

    def move(self, goal_pose):
        raise NotImplementedError(' move method does not implement. ')

    def openGripper(self):
        raise NotImplementedError(' openGripper method does not implement. ')

    def closeGripper(self):
        raise NotImplementedError(' closeGripper method does not implement. ')

    def rpy2orientation(self, row, pitch, yaw):
        q = tf.transformations.quaternion_from_euler(row, pitch, yaw, axes='sxyz')
        return q
