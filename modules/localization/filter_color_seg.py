import os
import sys
import numpy as np
import cv2


_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from modules.localization.localization import Localization

# accroding the rgb color space

class FilterColor(Localization):
    # set the color range, fliter the background
    # lower/upper:the color space in 3 channls
    # workspace: the interest area in the image,[xmin,ymin,xmax,ymax]
    def __init__(self, lower = np.array([10, 80, 0]),upper = np.array([80, 180, 80]),workspace=[100,1000,200,600]):
        self.lower = lower
        self.upper = upper
        self.xmin = workspace[0]
        self.xmax = workspace[2]
        self.ymin = workspace[1]
        self.ymax = workspace[3]

    # def SetColor(self,lower = np.array([10, 80, 0]),upper = np.array([80, 180, 80])):
    #     self.lower = lower
    #     self.upper = upper
    #     # the points' num of each contour
    # def SetContoursSize(self,min_num=100,max_num=500,min_area=1000):
    #     self.min = min_num
    #     self.max = max_num
    #     self.min_area = min_area
    # def workspace(self,workspace=[100,1000,200,600]):
    #     self.xmin = workspace[0]
    #     self.xmax = workspace[2]
    #     self.ymin = workspace[1]
    #     self.ymax = workspace[3]

    def run(self,image):
        crop_img = image[self.ymin:self.ymax,self.xmin:self.xmax,:] #[ymin:ymax,xmin:xmax]
        filtered_image = cv2.inRange(crop_img, self.lower, self.upper)
        img_medianBlur=cv2.medianBlur(filtered_image,5)

        mask = cv2.bitwise_not(img_medianBlur)
        width = currrentImg.shape[1]
        heigh = currrentImg.shape[0]


        result = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if(len(result)==2):
            contours = result[0]
            hierarchy = result[1]
        elif(len(result)==3):
            contours = result[1]
            hierarchy = result[2]
        area = []

        rect = [] #ymin,xmin,ymax,xmax
        for i in range(len(contours)):
            area_temp = cv2.contourArea(contours[i])
            if area_temp < 20*20 or area_temp > 200*200:
                continue
            else:
                temp = cv2.boundingRect(contours[i])
                temp = box + [self.xmin,self.ymin]
                rect.append(box_rate)
        #[[xmin,ymin,xmax,ymax],...]
        return None,rect,None


# function test
if __name__=='__main__':
    image = cv2.imread('xxx.jpg')
    lower = np.array([10, 80, 0])
    upper = np.array([80, 180, 80])
    workspace=[100,1000,200,600]
    filter = FilterColor(lower,upper,workspace)
    rect = filter.run(image)
    # show the rectangle in the image
    for i in range(len(rect)):
        cv2.rectangle(image,(int(rect[i][0]),int(rect[i][1])),(int(rect[i][2]),int(rect[i][3])),(0,255,0),2)

    cv2.imshow('result',image)
    cv2.waitKey()
    cv2.destroyAllWindows()
