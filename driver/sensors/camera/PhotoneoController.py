# Copyright (c) 2019 by Fang Wan. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8
# This is the driver for Photoneo and is dependent on PhoxiControl
# Please open PhoxiControl interface and connect to the Phoxi scanner before using this driver

import os
import sys
import numpy as np
import open3d as o3d

_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.append(_root_path)

from driver.sensors.camera.CameraController import CameraController, Frame


class PhotoneoController(CameraController):
    def __init__(self, configuration_path = None):
        super(PhotoneoController, self).__init__()
        self.width = 2064
        self.height = 1544

    def get_frame(self):
        os.system(_root_path+'/lib/Photoneo/Photoneo ' + _root_path+'/lib/Photoneo/ 0')
        pc = o3d.io.read_point_cloud(_root_path+"/lib/Photoneo/0.ply")
        # get texture image in gray scale between [0,1]
        texture_image = np.asarray(pc.colors).reshape([self.height, self.width, 3])

        # get depth image from z-axis point cloud, point cloud is organized according to the image sensor
        # For example, the pixel at position [x= 2010, y =350] in the texture is the (2064*349 + 2010) = 722346-th point in the point cloud.
        pointcloud_xyz = np.asarray(pc.points) # [n,3] in mm
        depth_image = np.asarray(pc.points)[:,2].reshape([self.height, self.width])

        return Frame([texture_image], [depth_image], [pointcloud_xyz], [None])


    def get_intrinsics(self):
        os.system("bash "+_root_path+"/lib/Photoneo/intrinsic_parameters.sh")
        data = []
        with open(_root_path+"/lib/Photoneo/intrinsic_parameters.txt") as fp:
            for cnt, line in enumerate(fp):
                if cnt in [2,4,6,7,15,16,17,18,19]:
                    data.append(float(line[:-1]))
        fp.close()

        return (data[0],data[2],data[1],data[3],data[4:])
