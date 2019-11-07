import os
import sys

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from input_output.publishers.AbstractSubject import AbstractSubject


class Publisher(AbstractSubject):
    '''A implementation of AbstractSubject class, used to collect image data and publish them to observers.
    :param
        observers: dict {'ob1': Monitor monitor1, 'ob2': Monitor monitor2, ...}
        data: dict {'Image': nparray image, ...}
    '''

    def __init__(self, name):
        self.name = name
        self.observers = {}
        self.data = {}

    def registerObserver(self, observer):
        self.observers[observer.name] = observer

    def removeObserver(self, observer):
        del self.observers[observer.name]

    def notifyObserver(self):
        for key in self.observers.keys():
            self.observers[key].update(self.data)

    def sendData(self, data):
        # if data.has_key("Time"):
        #     self.data["Time"] = data
        # if data.has_key("Image"):
        #     self.data["Image"] = data
        # if data.has_key["Grasp"]:
        #     self.data["Grasp"] = data
        self.data = data
        self.notifyObserver()
