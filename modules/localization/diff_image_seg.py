import os
import sys
import numpy as np
import cv2


_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from modules.localization.localization import Localization

DEBUG = False
class DiffFrame(Localization):
    # set a pre_collected image as background, detect the foreground
    def __init__(self,workspace=[100,1000,200,600]):
        self.xmin = workspace[0]
        self.xmax = workspace[2]
        self.ymin = workspace[1]
        self.ymax = workspace[3]

    def run(self,groundImg,currrentImg):
        groundImg_gray = cv2.cvtColor(groundImg,cv2.COLOR_BGR2GRAY)
        groundBlur = cv2.GaussianBlur(groundImg_gray,(5,5),1)
        groundBlur.dtype = 'int16'

        currrentImg_gray = cv2.cvtColor(currrentImg,cv2.COLOR_BGR2GRAY)
        currrentImgBlur = cv2.GaussianBlur(currrentImg_gray,(5,5),1)
        currrentImgBlur.dtype = 'int16'

        dGrayBlur = abs(groundBlur-currrentImgBlur)
        dGrayBlur.dtype = 'uint8'
        dGrayMidBlur=cv2.medianBlur(dGrayBlur,5)

        if DEBUG:
            cv2.imshow('diff image',dGrayMidBlur)
            cv2.waitKey()
        ret,thresh=cv2.threshold(dGrayMidBlur,20,255,cv2.THRESH_BINARY)
        if DEBUG:
            cv2.imshow('binary image',thresh)
            cv2.waitKey()
        result = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if(len(result)==2):
            contours = result[0]
            hierarchy = result[1]
        elif(len(result)==3):
            contours = result[1]
            hierarchy = result[2]
        if DEBUG:
            imag = cv2.drawContours(currrentImg,contours,-1,(0,255,0),3)
            cv2.imshow('contours image',imag)
            cv2.waitKey()


        rect = []
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area < 20*20 or area > 100*100:
                continue
            else:
                temp = cv2.boundingRect(contours[i])
                temp = list(temp)
                # x,y,w,h
                temp[2] = temp[2]+temp[0]
                temp[3] = temp[3]+temp[1]
                # the rects are in workspace
                if(temp[2]<self.xmin or temp[0]>self.xmax or temp[3]<self.ymin or temp[1]>self.ymax:
                    continue
                rect.append(temp)
        # [xmin,ymin,xmax,ymax]
        return None,rect,None


# function test
if __name__=='__main__':
    image = cv2.imread('xxx.jpg')
    filter = DiffFrame(lower,upper,workspace)
    rect = filter.run(image)
    # show the rectangle in the image
    for i in range(len(rect)):
        cv2.rectangle(image,(int(rect[i][0]),int(rect[i][1])),(int(rect[i][2]),int(rect[i][3])),(0,255,0),2)

    cv2.imshow('result',image)
    cv2.waitKey()
    cv2.destroyAllWindows()
