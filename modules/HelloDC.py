# Copyright (c) 2020 by BionicLab. All Rights Reserved.
# -*- coding:utf-8 -*-
"""
@File: HelloDC
@Author: Haokun Wang
@Date: 2020/3/26 14:34
@Description: 
"""
# import packages here


class HelloDC(object):
    def __init__(self, name: str):
        self.name = name

    def run(self, user_name: str):
        response = 'Hello, '+user_name+'! My name is '+self.name+'.'
        return response

# static functions here
