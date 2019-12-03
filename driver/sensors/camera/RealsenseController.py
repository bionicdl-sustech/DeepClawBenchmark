# Copyright (c) 2019 by Hank. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8
import os
import sys
import numpy as np
import pyrealsense2 as rs

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from driver.sensors.camera.CameraController import CameraController, Frame
from input_output.Configuration import *


class RealsenseController(CameraController):
    def __init__(self, configuration_path="/config/sensors/realsense.yaml"):
        super(RealsenseController, self).__init__()
        self._cfg = readConfiguration(configuration_path)
        self.width = self._cfg["FRAME_ARGS"]["width"]
        self.height = self._cfg["FRAME_ARGS"]["height"]
        self.fps = self._cfg["FRAME_ARGS"]["fps"]
        self.serial_id = self._cfg["DEVICE_CONFIGURATION"]["serial_id"]

        self.points = rs.points()
        self.pipeline = rs.pipeline()
        config = rs.config()
        if self.serial_id != '':
            config.enable_device(serial=self.serial_id)
        config.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, self.fps)
        config.enable_stream(rs.stream.infrared, 2, 1280, 720, rs.format.y8, self.fps)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)

        self.profile = self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()
        aligned_irL_frame = aligned_frames.get_infrared_frame(1)
        aligned_irR_frame = aligned_frames.get_infrared_frame(2)

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(aligned_color_frame.get_data())
        infrared_L = np.asanyarray(aligned_irL_frame.get_data())
        infrared_R = np.asanyarray(aligned_irR_frame.get_data())
        point_cloud = None

        return Frame([color_image], [depth_image], [point_cloud], [infrared_L, infrared_R])

    def get_intrinsics(self):
        color_stream = self.profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        return (intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy, [None])

    def get_device(self):
        return self.profile.get_device()

    def get_depth_scale(self):
        depth_sensor = self.profile.get_device().first_depth_sensor()
        return depth_sensor.get_depth_scale()

    def get_serial_number(self):
        device = self.get_device()
        return str(device.get_info(rs.camera_info.serial_number))


