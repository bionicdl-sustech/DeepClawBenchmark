# Copyright (c) 2019 by liuxiaobo. All Rights Reserved.
# !/usr/bin/python
# coding=utf-8

import cv2
import numpy as np
import random
import time
import math

import sys
import os

DEBUG = False
'''
the coordinate of the image is
        ----------------> x
        |
        |
        |
       \|/
        y
the returned box is (x,y)
'''

class ComputeArea(object):
    #only compute the boundingbox in the target region
    #Todo: if the range gived is out of the image size, change them the image size
    def __init__(self,xmin=330,ymin=440,xmax=690,ymax=720):
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        self.SetColor()
        self.SetContoursSize()
    # segment the object with color
    # Note: the color space of opencv is BGR in default
    def SetColor(self,lower = np.array([10, 80, 0]),upper = np.array([80, 180, 80])):
        self.lower = lower
        self.upper = upper
        # the points' num of each contour
    def SetContoursSize(self,min_num=100,max_num=500,min_area=1000):
        self.min = min_num
        self.max = max_num
        self.min_area = min_area

    def FilterColor(self,img):
        mask = cv2.inRange(img, self.lower, self.upper)
        mask = cv2.bitwise_not(mask)
        return mask

    def GetBoundingBox(self,img):
        # crop the image
        crop_img = img[self.ymin:self.ymax,self.xmin:self.xmax]
        if(DEBUG):
            cv2.imshow('crop',crop_img)
            cv2.waitKey()
        mask = self.FilterColor(crop_img)
        if(DEBUG):
            cv2.imshow('mask',mask)
            cv2.waitKey()

        result = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if(len(result)==2):
            contours = result[0]
            hierarchy = result[1]
        elif(len(result)==3):
            contours = result[1]
            hierarchy = result[2]
        showed_image = crop_img.copy()
        rect = []
        new_contours = []
        for i, c in enumerate(contours):
            color = (random.randint(0,256), random.randint(0,256), random.randint(0,256))
            # contour: if hierarchy[0][i][3] == -1, it means there are contours inside
            # print(len(contours[i]))
            # print(cv2.contourArea(contours[i]))
            if (len(contours[i]) < self.min or len(contours[i]) > self.max or cv2.contourArea(contours[i]) < self.min_area): #or hierarchy[0][i][3] == -1
                continue
            new_contours.append(contours[i])
            cv2.drawContours(showed_image, contours, i, color)
            rect.append(cv2.minAreaRect(c))
        # add all contours as one
        if(len(new_contours) == 0):
            print('No Objects Detected!')
            return None
        final_contour = new_contours[0]
        for i in range(1,len(new_contours)):
            final_contour = np.append(final_contour,new_contours[i],axis=0)
        # the rect points are clockwise
        min_Box = cv2.minAreaRect(final_contour)
        box = cv2.boxPoints(min_Box)
        box = np.intp(box) #np.intp: Integer used for indexing (same as C ssize_t; normally either int32 or int64)

        if(DEBUG):
            cv2.drawContours(showed_image, [box], -1, (255,0,0),2)
            # cv2.drawContours(showed_image,new_contours,-1,(255,0,0))
            print('rect: ',min_Box)
            cv2.imshow('contours',showed_image)
            cv2.waitKey()
            cv2.destroyAllWindows()
        return box + np.array([self.xmin,self.ymin])

    def GetBoardBox(self,img):
        # crop the image
        crop_img = img[self.ymin:self.ymax,self.xmin:self.xmax]
        if(DEBUG):
            cv2.imshow('crop',crop_img)
            cv2.waitKey()
        mask = self.FilterColor(crop_img)
        if(DEBUG):
            cv2.imshow('mask',mask)
            cv2.waitKey()
        # contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        result = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if(len(result)==2):
            contours = result[0]
            hierarchy = result[1]
        elif(len(result)==3):
            contours = result[1]
            hierarchy = result[2]

        showed_image = crop_img.copy()
        rect = []
        new_contours = []
        for i, c in enumerate(contours):
            color = (random.randint(0,256), random.randint(0,256), random.randint(0,256))
            # contour: if hierarchy[0][i][3] == -1, it means there are contours inside
            if (len(contours[i]) < self.min or len(contours[i]) > self.max or cv2.contourArea(contours[i]) < self.min_area): #or hierarchy[0][i][3] == -1
                continue
            cv2.drawContours(showed_image, contours, i, color)
            min_Box = cv2.minAreaRect(c)
            if(min_Box[1][0] >80 and min_Box[1][1] >80):
                box = cv2.boxPoints(min_Box)
                box = np.intp(box) #np.intp: Integer used for indexing (same as C ssize_t; normally either int32 or int64)
                cv2.drawContours(showed_image, [box], -1, (255,0,0),2)
                if(DEBUG):
                    print('rect: ',min_Box)
                    cv2.imshow('contours',showed_image)
                    cv2.waitKey()
                    cv2.destroyAllWindows()

                x,y = min_Box[0]
                x = x + self.xmin
                y = y + self.ymin
                angle = min_Box[2] * 3.14 /180.0
                return x,y,angle


if __name__ =='__main__':
    current_path = os.path.dirname(os.path.abspath(__file__))
    root_path = os.path.dirname(os.path.dirname(current_path))
    sys.path.append(root_path)
    from driver.sensors.camera.RealsenseController import RealsenseController

    frame= self.camera.get_frame()
    color_image_background = frame.color_image[0]
    time.sleep(1)
    frame= self.camera.get_frame()
    color_image_background = frame.color_image[0]

    place_box = [700,150,950,550]#ur5 [800,200,1020,560]
    area_compute = ComputeArea(place_box[0], place_box[1], place_box[2], place_box[3])

    area_compute.SetColor(np.array([100, 80, 80]),np.array([180, 180, 180]))
    area_compute.SetContoursSize(min_num = 100, max_num = 1500, min_area = 1000)
    compute_area = False
    if compute_area:
        points_area = area_compute.GetBoundingBox(color_image)
        cv2.drawContours(color_image, [points_area], -1, (255,0,0),2)
        cv2.imshow('result',color_image)
        cv2.waitKey()
        cv2.destroyAllWindows()
        exit()

    board_area = area_compute.GetBoardBox(color_image)
    print(board_area)
    x,y,angle = board_area
    xf = 25.0
    yf = 25.0
    p3 = [0,0,angle]
    p3[0] = x + (-1)*xf*math.sin(angle) + (-1)*yf*math.cos(angle)
    p3[1] = y - (-1)*xf*math.cos(angle) + (-1)*yf*math.sin(angle)

    p2 = [0,0,angle]
    p2[0] = x + xf*math.sin(angle) + (-1)*yf*math.cos(angle)
    p2[1] = y - xf*math.cos(angle) + (-1)*yf*math.sin(angle)

    p1 = [0,0,angle]
    p1[0] = x + (-1)*xf*math.sin(angle) + yf*math.cos(angle)
    p1[1] = y - (-1)*xf*math.cos(angle) + yf*math.sin(angle)

    p0 = [0,0,angle]
    p0[0] = x + xf*math.sin(angle) + yf*math.cos(angle)
    p0[1] = y - xf*math.cos(angle) + yf*math.sin(angle)

    cv2.circle(color_image, (int(p0[0]), int(p0[1])), 5, (255,0,0), 4)
    cv2.circle(color_image, (int(p1[0]), int(p1[1])), 5, (0,255,0), 4)
    cv2.circle(color_image, (int(p2[0]), int(p2[1])), 5, (0,0,255), 4)
    cv2.circle(color_image, (int(p3[0]), int(p3[1])), 5, (0,188,110), 4)


    cv2.imshow('result',color_image)
    cv2.waitKey()
    cv2.destroyAllWindows()
