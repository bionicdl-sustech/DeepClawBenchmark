import cv2
import os
import sys
import time

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from input_output.observers.AbstractObserver import AbstractObserver


class ImageMonitor(AbstractObserver):
    def __init__(self, name):
        self.name = name
        self.data = {}
        self.dir = ''
        self.img_name = ''

    def update(self, data):
        self.data = data
        self.display()

    def display(self):
        # if self.data.has_key('Image'):
        if "Image" in self.data:
            if not os.path.exists(self.dir):
                os.makedirs(self.dir)
            cv2.imwrite(self.dir + self.img_name, self.data['Image'])
