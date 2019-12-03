import os
import sys
import cv2
import numpy as np

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from modules.recognition.recognition import Recognition


class color_recognition(Recognition):
    def __init__(self, low_hsv, high_hsv, size=5):
        self.size = size
        self.low_hsv = np.array(low_hsv)
        self.high_hsv = np.array(high_hsv)

    def display(self, centers, color_image, **kwargs):
        if len(centers) != 0:
            labels = []
            for center in centers:
                piece_image = color_image[center[1]-self.size:center[1]+self.size, center[0]-self.size:center[0]+self.size, :]
                piece_hsv = cv2.cvtColor(piece_image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(piece_hsv, lowerb=self.low_hsv, upperb=self.high_hsv)
                mask[mask==255] = 1
                if np.sum(mask) >= 0.6*self.size**2:
                    labels.append(1)
                else:
                    labels.append(0)
            return labels, None
        return None, None
