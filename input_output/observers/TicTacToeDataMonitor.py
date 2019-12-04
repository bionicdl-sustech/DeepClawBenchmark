import csv
import os
import sys
import time

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from input_output.observers.AbstractObserver import AbstractObserver


class TicTacToeDataMonitor(AbstractObserver):
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
        if "TicTacToeData" in self.data:
            if not os.path.exists(self.dir+self.csv_name):
                if not os.path.exists(self.dir):
                    os.makedirs(self.dir)
                file_path = self.dir + self.csv_name
                csvFile = open(file_path, "a")
                writer = csv.writer(csvFile)
                writer.writerow(["bounding_box", "centers", "labels", "pose"])
                csvFile.close()

            bounding_box = self.data['TicTacToeData'][0]
            centers = self.data['TicTacToeData'][1]
            labels = self.data['TicTacToeData'][2]
            pose = self.data['TicTacToeData'][3]

            if bounding_box is None:
                bounding_box = "None"
            if centers is None:
                centers = "None"
            if labels is None:
                labels = "None"
            if pose is None:
                pose = "None"

            file_path = self.dir + self.csv_name
            csvFile = open(file_path, "a")
            writer = csv.writer(csvFile)
            writer.writerow([bounding_box, centers, labels, pose])
            csvFile.close()
