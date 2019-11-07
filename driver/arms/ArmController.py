# Copyright (c) 2019 by Hank. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8
import tf
import numpy as np


class ArmController(object):
    def __init__(self):
        pass

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

    def get_rigid_transform(self, A, B):
        assert len(A) == len(B)
        N = A.shape[0]  # Total points
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - np.tile(centroid_A, (N, 1))  # Centre the points
        BB = B - np.tile(centroid_B, (N, 1))
        H = np.dot(np.transpose(AA), BB)  # Dot is matrix multiplication for array
        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        if np.linalg.det(R) < 0:  # Special reflection case
            Vt[2, :] *= -1
            R = np.dot(Vt.T, U.T)
        t = np.dot(-R, centroid_A.T) + centroid_B.T
        return R, t
