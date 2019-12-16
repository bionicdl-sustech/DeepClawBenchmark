import csv
import os
import sys
import time

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from input_output.observers.AbstractObserver import AbstractObserver

class JigsawDataMonitor(AbstractObserver):
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
        if "JigsawData" in self.data:
            if not os.path.exists(self.dir+self.csv_name):
                if not os.path.exists(self.dir):
                    os.makedirs(self.dir)
                file_path = self.dir + self.csv_name
                csvFile = open(file_path, "a")
                writer = csv.writer(csvFile)
                writer.writerow(["ground_truth","bounding_box", "labels", "pickpose","placepose","score"])
                csvFile.close()

            ground_truth = self.data['JigsawData'][0]
            bounding_box = self.data['JigsawData'][1]
            labels = self.data['JigsawData'][2]
            pickpose = self.data['JigsawData'][3]
            placepose = self.data['JigsawData'][4]
            score = self.data['JigsawData'][5]

            if bounding_box is None:
                bounding_box = "None"
            if ground_truth is None:
                ground_truth = "None"
            if labels is None:
                labels = "None"
            if pickpose is None:
                pickpose = "None"
            if placepose is None:
                placepose = "None"
            if score is None:
                score = "None"

            file_path = self.dir + self.csv_name
            csvFile = open(file_path, "a")
            writer = csv.writer(csvFile)
            writer.writerow([ground_truth, bounding_box, labels, pickpose,placepose,score])
            csvFile.close()
