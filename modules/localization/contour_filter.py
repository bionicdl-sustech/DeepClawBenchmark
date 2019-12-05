import os
import sys
import cv2
import numpy as np

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from modules.localization.localization import Localization


class contour_filter(Localization):
    def __init__(self, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_NONE, area_threshold=None, minAreaBox=False):
        if area_threshold is None:
            area_threshold = [80, 200]
        self.mode = mode
        self.method = method
        self.area_threshold = area_threshold
        self.minAreaBox = minAreaBox

    def display(self, color_image, **kwargs):
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray_image, 127, 255, 0)
        cv2.imshow("thresh", thresh)
        cv2.waitKey(1)
        raw_contours, hierarchy = cv2.findContours(thresh, self.mode, self.method)

        # area filter
        contours = []
        for contour in raw_contours:
            area = self.calculate_area(contour)
            if self.area_threshold[0] < area < self.area_threshold[1]:
                contours.append(contour)

        # get mask
        mask = np.zeros(thresh.shape)
        cv2.drawContours(mask, contours, -1, 1, -1)

        # calculate centers and bounding boxes
        centers = []
        bounding_boxes = []
        for contour in contours:
            centers.append(self.find_center(contour))
            bounding_boxes.append(self.find_bounding_box(contour, is_minArea=self.minAreaBox))

        return bounding_boxes, mask, centers

    def find_center(self, contour):
        M = cv2.moments(contour)
        center_x = int(M["m10"] / M["m00"])
        center_y = int(M["m01"] / M["m00"])
        return [center_x, center_y]

    def find_bounding_box(self, contour, is_minArea=False):
        if is_minArea:
            # ((x, y), (w, h), theta)
            rect = cv2.minAreaRect(contour)
            return rect
        else:
            x, y, w, h = cv2.boundingRect(contour)
            x = x + int(w/2.0)
            y = y + int(h/2.0)
            return ((x, y), (w, h), 0)

    def calculate_area(self, contour):
        area = cv2.contourArea(contour)
        return area