# Copyright (c) 2020 by BionicLab. All Rights Reserved.
# -*- coding:utf-8 -*-
"""
@File: ContourDetector
@Author: Haokun Wang
@Date: 2020/3/24 10:30
@Description: 
"""

import cv2
import numpy as np


class ContourDetector(object):
    def __init__(self, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_NONE,
                 binary_threshold=127, area_threshold=(80, 200), with_angle=False):
        self.mode = mode
        self.method = method
        self.binary_threshold = binary_threshold
        self.area_threshold_min, self.area_threshold_max = area_threshold
        self.with_angle = with_angle

    def run(self, color_image):
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray_image, self.binary_threshold, 255, 0)
        raw_contours, hierarchy = cv2.findContours(thresh, self.mode, self.method)

        # area filter
        contours = []
        for contour in raw_contours:
            if self.area_threshold_min < cv2.contourArea(contour) < self.area_threshold_max:
                contours.append(contour)

        # get mask
        mask = np.zeros(thresh.shape)
        cv2.drawContours(mask, contours, -1, 1, -1)

        # calculate centers and bounding boxes
        centers = [find_center(contour) for contour in contours]
        bounding_boxes = [find_bounding_box(contour, self.with_angle) for contour in contours]

        return bounding_boxes, mask, centers


def find_center(contour):
    M = cv2.moments(contour)
    center_x = int(M["m10"] / M["m00"])
    center_y = int(M["m01"] / M["m00"])
    return [center_x, center_y]


def find_bounding_box(contour, with_angle=False):
    if with_angle:
        # ((x, y), (w, h), theta)
        rect = cv2.minAreaRect(contour)
        return rect
    else:
        x, y, w, h = cv2.boundingRect(contour)
        x = x + int(w/2.0)
        y = y + int(h/2.0)
        return (x, y), (w, h), 0
