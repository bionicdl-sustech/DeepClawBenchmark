import cv2
import os
import sys
import time

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_path)

class AbstractSubject(object):
    '''An abstract subject class.
    Provide abstract register, remove and notify methods, all publisher class should inherit this class.
    '''
    def registerObserver(self):
        raise NotImplementedError(
            ' registerObserver method does not implement. ')

    def removeObserver(self):
        raise NotImplementedError(
            ' removeObserver method does not implement. ')

    def notifyObserver(self):
        raise NotImplementedError(
            ' notifyObserver method does not implement. ')

class AbstractObserver(object):
    '''An abstract observer class.
    Provide abstract updata, display methods, all observer class should inherit this class.
    '''
    def update(self):
        raise NotImplementedError(
            ' update method does not implement. ')

    def display(self):
        raise NotImplementedError(
            ' display method does not implement. ')

class TimePublisher(AbstractSubject):
    '''A implementation of AbstractSubject class, used to collect image data and publish them to observers.
    :param
        observers: dict {'ob1': Monitor monitor1, 'ob2': Monitor monitor2, ...}
        data: dict {'Image': nparray image, ...}
    '''

    def __init__(self, name):
        self.name = name
        self.observers = {}
        self.data = {'Time': ''}

    def registerObserver(self, observer):
        self.observers[observer.name] = observer

    def removeObserver(self, observer):
        del self.observers[observer.name]

    def notifyObserver(self):
        for key in self.observers.keys():
            self.observers[key].update(self.data)

    def setData(self, time):
        self.data['Time'] = time
        self.notifyObserver()

class ImagePublisher(AbstractSubject):
    '''A implementation of AbstractSubject class, used to collect image data and publish them to observers.
    :param
        observers: dict {'ob1': Monitor monitor1, 'ob2': Monitor monitor2, ...}
        data: dict {'Image': nparray image, ...}
    '''

    def __init__(self, name):
        self.name = name
        self.observers = {}
        self.data = {'Image': ''}

    def registerObserver(self, observer):
        self.observers[observer.name] = observer

    def removeObserver(self, observer):
        del self.observers[observer.name]

    def notifyObserver(self):
        for key in self.observers.keys():
            self.observers[key].update(self.data)

    def setData(self, image):
        self.data['Image'] = image
        self.notifyObserver()

class Monitor(AbstractObserver):
    def __init__(self, name):
        self.name = name
        self.data = {}

    def update(self, data):
        self.data = data
        self.display()

    def display(self):
        print(self.data)
        # path = root_path+'/Data/'+time.strftime("%Y%m%d-%H%M%S", time.localtime())
        # stamp = int(time.time())
        # if not os.path.exists(path):
        #     os.makedirs(path)
        # cv2.imwrite(path+'/'+str(stamp)+'.jpg', self.data['Image'])
        # # cv2.imshow('Monitor', self.data['Image'])

