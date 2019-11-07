import csv
import os
import sys
import time

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from input_output.observers.AbstractObserver import AbstractObserver


class GraspMonitor(AbstractObserver):
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

