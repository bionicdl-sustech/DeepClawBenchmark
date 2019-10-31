import csv
import os
import sys
import time

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from input_output.observers.AbstractObserver import AbstractObserver


class TimeMonitor(AbstractObserver):
    def __init__(self, name):
        self.image_counter = 0
        self.name = name
        self.data = {}

    def update(self, data):
        self.data = data
        self.display()

    def display(self):
        path = _root_path+'/Data/'+self.name
        stamp = int(time.time())
        if not os.path.exists(path):
            os.makedirs(path)

        if self.data.has_key('Time'):
            file_path = path+'/'+self.name+'_time.csv'
            name = self.data['Time'][0]
            executing_time = self.data['Time'][1]
            csvFile = open(file_path, "a")
            writer = csv.writer(csvFile)
            writer.writerow([name, executing_time])
            csvFile.close()
            # return [name, executing_time]
