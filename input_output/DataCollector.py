import cv2
import csv
import os
import sys
import time

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)


class AbstractSubject(object):
    '''An abstract subject class.
    Provide abstract register, remove and notify methods, all publisher class should inherit this class.
    '''
    def registerObserver(self, observer):
        raise NotImplementedError(
            ' registerObserver method does not implement. ')

    def removeObserver(self, observer):
        raise NotImplementedError(
            ' removeObserver method does not implement. ')

    def notifyObserver(self):
        raise NotImplementedError(
            ' notifyObserver method does not implement. ')

class AbstractObserver(object):
    '''An abstract observer class.
    Provide abstract updata, display methods, all observer class should inherit this class.
    '''
    def update(self, data):
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

class GraspingDataPublisher(AbstractSubject):
    '''A implementation of AbstractSubject class, used to collect image data and publish them to observers.
    :param
        observers: dict {'ob1': Monitor monitor1, 'ob2': Monitor monitor2, ...}
        data: dict {'Image': nparray image, ...}
    '''

    def __init__(self, name):
        self.name = name
        self.observers = {}
        self.data = {'Grasp': []}

    def registerObserver(self, observer):
        self.observers[observer.name] = observer

    def removeObserver(self, observer):
        del self.observers[observer.name]

    def notifyObserver(self):
        for key in self.observers.keys():
            self.observers[key].update(self.data)

    def setData(self, position, angel, label, probability, successful_rate, precision):
        self.data['Grasp'] = [position, angel, label, probability, successful_rate, precision]
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
        self.image_counter = 0
        self.name = name
        self.data = {}

    def update(self, data):
        self.data = data
        self.display()

    def display(self):
        path = _root_path+'/data/'+self.name
        stamp = int(time.time())
        if not os.path.exists(path):
            os.makedirs(path)

        if self.data.has_key('Image'):
            if not os.path.exists(path + '/images'):
                os.makedirs(path + '/images')
            cv2.imwrite(path+'/images/' + self.name + str(self.image_counter)+'.jpg', self.data['Image'])
            self.image_counter += 1
            # return self.data['Image']

        if self.data.has_key('Time'):
            file_path = path+'/'+self.name+'_time.csv'
            name = self.data['Time'][0]
            executing_time = self.data['Time'][1]
            csvFile = open(file_path, "a")
            writer = csv.writer(csvFile)
            writer.writerow([name, executing_time])
            csvFile.close()
            # return [name, executing_time]

        if self.data.has_key('Grasp'):
            file_path = path+'/'+self.name+'_grasp.csv'
            if not os.path.exists(file_path):
                csvFile = open(file_path, "a")
                writer = csv.writer(csvFile)
                writer.writerow(['u', 'v', 'x', 'y', 'z', 'angel', 'label', 'probability', 'successful_rate', 'precision'])
                csvFile.close()
            position = self.data['Grasp'][0]
            angle = self.data['Grasp'][1]
            label = self.data['Grasp'][2]
            probability = self.data['Grasp'][3]
            successful_rate = self.data['Grasp'][4]
            precision = self.data['Grasp'][5]

            csvFile = open(file_path, "a")
            writer = csv.writer(csvFile)
            writer.writerow([position[0], position[1], position[2], position[3], position[4], angle, label,
                             probability, successful_rate, precision])
            csvFile.close()

