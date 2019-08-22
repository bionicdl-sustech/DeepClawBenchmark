# Copyright (c) 2019 by liuxiaobo. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8
import numpy as np
import pyrealsense2 as rs
import time
import cv2

class RealsenseController(object):
    def __init__(self, width = 1280, hight = 720,fps = 30):
        self.width = width
        self.hight = hight
        self.fps = fps

        self.points = rs.points()
        self.pipeline= rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.infrared, 1, self.width, self.hight, rs.format.y8, self.fps)
        config.enable_stream(rs.stream.infrared, 2, self.width, self.hight, rs.format.y8, self.fps)
        config.enable_stream(rs.stream.depth, self.width, self.hight, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.width, self.hight, rs.format.bgr8, self.fps)

        profile = self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def getImage(self):
        frames = self.pipeline.wait_for_frames()
        irL_frame = frames.get_infrared_frame(1)
        irR_frame = frames.get_infrared_frame(2)
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if(not aligned_depth_frame or not color_frame):
            return None, None, None, None
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        image_L = np.asanyarray(irL_frame.get_data())
        image_R = np.asanyarray(irR_frame.get_data())
        return color_image,depth_image,image_L,image_R


if __name__ == '__main__':
    camera_controller = RealsenseController()
    color_image,_,_,_ = camera_controller.getImage()
    cv2.imwrite('dog_stand_redbox_ref.png',color_image)
