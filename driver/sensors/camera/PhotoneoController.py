# Copyright (c) 2019 by Hank. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8
import os
import sys
import numpy as np
import open3d as o3d

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from driver.sensors.camera.CameraController import CameraController


class PhotoneoController(CameraController):
    def __init__(self):
        super(PhotoneoController, self).__init__()
        self.width = 2064
        self.height = 1544

    def getImage(self):
        os.system(r'./lib/Photoneo/Photoneo ./lib/Photoneo/ 0')
        pc = o3d.io.read_point_cloud("./lib/Photoneo/0.ply")
        # get texture image in gray scale between [0,1]
        texture_image = np.asarray(pc.colors).reshape([self.height, self.width, 3])

        # get depth image from z-axis point cloud, point cloud is organized according to the image sensor
        # For example, the pixel at position [x= 2010, y =350] in the texture is the (2064*349 + 2010) = 722346-th point in the point cloud.
        pointcloud_xyz = np.asarray(pc.points) # [n,3] in mm
        depth_image = np.asarray(pc.points)[:,2].reshape([self.height, self.width])

        return ([texture_image], [depth_image], [None], [pointcloud_xyz])


    def get_intrinsics(self):
        os.system("bash ./lib/Photoneo/intrinsic_parameters.sh")
        data = []
        with open("./lib/Photoneo/intrinsic_parameters.txt") as fp:
            for cnt, line in enumerate(fp):
                if cnt in [2,4,6,7,15,16,17,18,19]:
                    data.append(float(line[:-1]))

        fp.close()
        return (data[0],data[2],data[1],data[3],data[4:])
