import cv2
import os
import sys
import time

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from input_output.observers.AbstractObserver import AbstractObserver


class ImageMonitor(AbstractObserver):
    def __init__(self, name):
        self.image_counter = 0
        self.name = name
        self.data = {}

    def update(self, data):
        self.data = data
        self.display()

    def display(self):
        path = _root_path + '/Data/' + self.name
        stamp = int(time.time())
        if not os.path.exists(path):
            os.makedirs(path)

        if self.data.has_key('Image'):
            if not os.path.exists(path + '/images'):
                os.makedirs(path + '/images')
            cv2.imwrite(path + '/images/' + self.name + str(self.image_counter) + '.jpg', self.data['Image'])
            self.image_counter += 1
            # return self.data['Image']
