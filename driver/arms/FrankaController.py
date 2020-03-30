# Copyright (c) 2020 by BionicDL Lab. All Rights Reserved.
# -*- coding:utf-8 -*-
"""
@File: FrankaController
@Author: Haokun Wang
@Date: 2020/3/17 15:45
@Description: 
"""
import sys
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(ROOT)

from driver.arms.ArmController import ArmController


class FrankaController(ArmController):
    def __init__(self, configuration_file):
        super(FrankaController, self).__init__()
        self.cfg = configuration_file
        self.__home_pose = self.cfg['HOME_POSE']
        self.__home_joints = self.cfg['HOME_JOINTS']

    def move_j(self, *args, **kwargs):
        # TODO: wait for update
        pass

    def move_p(self, *args, **kwargs):
        # TODO: wait for update
        pass

    def get_state(self, *args, **kwargs):
        # TODO: wait for update
        pass

    def verify_state(self, *args, **kwargs):
        # TODO: wait for update
        pass

