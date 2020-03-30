# Copyright (c) 2020 by Hank. All Rights Reserved.
# -*- coding:utf-8 -*-
"""
@File: EyeOnBase
@Author: Haokun Wang
@Date: 2020/3/18 10:46
@Description: 
"""
import numpy as np
import cv2
import os


def image_callback(color_image, depth_image, intrinsics):
    checkerboard_size = (3, 3)
    refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    fx = intrinsics[0]
    fy = intrinsics[1]
    cx = intrinsics[2]
    cy = intrinsics[3]

    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    checkerboard_found, corners = cv2.findChessboardCorners(gray, checkerboard_size, None,
                                                            cv2.CALIB_CB_ADAPTIVE_THRESH)
    if checkerboard_found:
        corners_refined = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), refine_criteria)

        # Get observed checkerboard center 3D point in camera space
        checkerboard_pix = np.round(corners_refined[4, 0, :]).astype(int)
        checkerboard_z = np.mean(np.mean(depth_image[checkerboard_pix[1] - 20:checkerboard_pix[1] + 20,
                                         checkerboard_pix[0] - 20:checkerboard_pix[0] + 20])) / 1000.0
        checkerboard_x = np.multiply(checkerboard_pix[0] - cx, checkerboard_z / fx)  # 1920, 1080
        checkerboard_y = np.multiply(checkerboard_pix[1] - cy, checkerboard_z / fy)  # 1920, 1080
        print("Found checkerboard, X,Y,Z = ", [checkerboard_x, checkerboard_y, checkerboard_z])
        if checkerboard_z > 0:
            # Save calibration point and observed checkerboard center
            observed_pt = np.array([checkerboard_x, checkerboard_y, checkerboard_z])
            return observed_pt
    return []


class Calibration(object):
    def __init__(self, arm, camera, configuration_file):
        self.__arm = arm
        self.__camera = camera
        self.__cfg = configuration_file

    def run(self):
        initial_pose = self.__cfg['initial_position']
        x_step = self.__cfg['x_stride']
        y_step = self.__cfg['y_stride']
        z_step = self.__cfg['z_stride']

        self.__arm.move_p(initial_pose)
        x = initial_pose[0]
        y = initial_pose[1]
        z = initial_pose[2]

        observed_pts = []
        measured_pts = []
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    self.__arm.move_p([x + x_step * i, y + y_step * j, z + z_step * k,
                                       initial_pose[3], initial_pose[4], initial_pose[5]])
                    # time.sleep(0.5)
                    frame = self.__camera.get_frame()
                    color_image = frame.color_image[0]
                    depth_image = frame.depth_image[0]
                    observed_pt = image_callback(color_image, depth_image, self.__camera.get_intrinsics())
                    measured_pt = [x + x_step * i, y + y_step * j, z + z_step * k + self.__cfg["OFFSET"]]
                    print(measured_pt)
                    if len(observed_pt) != 0:
                        observed_pts.append(observed_pt)
                        measured_pts.append(measured_pt)
        root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        np.savez(os.path.dirname(root_path) + self.__cfg["CALIBRATION_DIR"], observed_pts, measured_pts)

