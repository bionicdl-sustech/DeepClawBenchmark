import os
import sys
import cv2
import numpy as np
import time
_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(_root_path)

from modules.localization.localization import Localization
from driver.sensors.camera.RealsenseController import RealsenseController

class ContourFilter(Localization):
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
        # cv2.imshow("thresh", thresh)
        # cv2.waitKey(1)
        raw_contours, hierarchy = cv2.findContours(thresh, self.mode, self.method)

        # area filter
        contours = []
        for contour in raw_contours:
            area = calculate_area(contour)
            if self.area_threshold[0] < area < self.area_threshold[1]:
                contours.append(contour)

        # get mask
        mask = np.zeros(thresh.shape)
        cv2.drawContours(mask, contours, -1, 1, -1)

        # calculate centers and bounding boxes
        centers = []
        bounding_boxes = []
        for contour in contours:
            centers.append(find_center(contour))
            bounding_boxes.append(find_bounding_box(contour, is_minArea=self.minAreaBox))

        return bounding_boxes, mask, centers


def find_center(contour):
    M = cv2.moments(contour)
    center_x = int(M["m10"] / M["m00"])
    center_y = int(M["m01"] / M["m00"])
    return [center_x, center_y]


def find_bounding_box(contour, is_minArea=False):
    if is_minArea:
        # ((x, y), (w, h), theta)
        rect = cv2.minAreaRect(contour)
        return rect
    else:
        x, y, w, h = cv2.boundingRect(contour)
        x = x + int(w/2.0)
        y = y + int(h/2.0)
        return (x, y), (w, h), 0


def calculate_area(contour):
    area = cv2.contourArea(contour)
    return area

if __name__ == '__main__':
    realsense = RealsenseController("/config/sensors/realsense.yaml")
    get_frame = realsense.get_frame()
    time.sleep(1)
    get_frame = realsense.get_frame()
    img = get_frame.color_image[0]
    img = img [300:700,450:950]
    contour_filter = ContourFilter(area_threshold = [50,150000],minAreaBox = True)
    bounding_boxes, mask, centers = contour_filter.display(img)
    a = []
    for i in range(len(bounding_boxes)):
        a.append(bounding_boxes[i][1][0]*bounding_boxes[i][1][1])
        # cv2.rectangle(img,(bounding_boxes[i][0][0],bounding_boxes[i][0][1]),(bounding_boxes[i][0][0]+bounding_boxes[i][1][0],bounding_boxes[i][0][1]+bounding_boxes[i][1][1]),(0,255,0),3)
    idex = a.index(max(a))
    box = cv2.boxPoints(bounding_boxes[idex])
    print(centers[idex])
    print(bounding_boxes[idex])
    box_d = np.int0(box)
    cv2.drawContours(img, [box_d], 0, (0,255,0), 3)
    cv2.rectangle(img,(centers[idex][0],centers[idex][1]),(centers[idex][0],centers[idex][1]),(0,255,0),3)
    cv2.imshow('img',img)
    cv2.waitKey()
