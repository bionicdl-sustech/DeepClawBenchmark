import numpy as np
import cv2

class CVAlgorithm:
    def __init__(self, name='', id=0):
        self.name = name
        self.id = id

    def color2gray(self, color_image):
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
        return gray_image

    def color2binary(self, color_image, threshold=127, target_value=255, model=cv2.THRESH_BINARY):
        gray_image = self.color2gray(color_image)
        ret, binary_image = cv2.threshold(gray_image, threshold, target_value, model)
        return binary_image

    def find_contours(self, color_image, threshold=127):
        binary_image = self.color2binary(color_image, threshold)
        _, contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def contours_area_filter(self, contours, min_area, max_area):
        eligible_contours=[]
        for contour in contours:
            area = cv2.contourArea(contour)
            if min_area <= area and max_area >= area:
                eligible_contours.append(contour)
        return eligible_contours

    def find_contours_center(self, contours):
        centers = []
        for contour in contours:
            M = cv2.moments(contour)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            centers.append([cx, cy])
        return centers

    def find_min_area_rectangle(self, contour):
        rectangle = cv2.minAreaRect(contour)
        box = cv2.BoxPoints(rectangle)
        return rectangle, box

    def color_mask(self, color_image, lower_color, upper_color):
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)
        mask_image = cv2.bitwise_and(color_image, color_image, mask=mask)
        return mask_image

    def crop_mask(self, color_image, region):
        mask = np.zeros(color_image.shape[0:2], dtype="uint8")
        roi = cv2.rectangle(mask, (region[0], region[1]), (region[2], region[3]),
                      255, -1)
        return cv2.add(color_image,
                       np.zeros(np.shape(color_image), dtype=np.uint8),
                       mask=roi)

    def match_template(self, background, template):
        gray_background = self.color2gray(background)
        w, h = template.shape[::-1]
        res = cv2.matchTemplate(gray_background, template, cv2.TM_CCOEFF_NORMED) #4
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        top_left = max_loc
        bottom_right = (top_left[0] + w, top_left[1] + h)
        cv2.rectangle(background, top_left, bottom_right, [0, 0, 255], 1)
        return background, top_left, bottom_right

