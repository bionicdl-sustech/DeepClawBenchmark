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
        self.dir = ''
        self.csv_name = ''

    def update(self, data):
        self.data = data
        self.display()

    def display(self):
        # if self.data.has_key('Time'):
        if "Time" in self.data:
            if not os.path.exists(self.dir):
                os.makedirs(self.dir)

            name = self.data['Time'][0]
            executing_time = self.data['Time'][1]

            file_path = self.dir + self.csv_name
            csvFile = open(file_path, "a")
            writer = csv.writer(csvFile)
            writer.writerow([name, executing_time])
            csvFile.close()
