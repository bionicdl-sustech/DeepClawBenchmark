import os
import sys

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from input_output.observers.AbstractObserver import AbstractObserver


class PCLMonitor(AbstractObserver):
    def __init__(self, name):
        self.name = name
        self.data = {}
        self.dir = ""
        self.pcl_name = ""

    def update(self, data):
        self.data = data
        self.display()

    def display(self):
        if "PCL" in self.data:
            if not os.path.exists(self.dir):
                os.makedirs(self.dir)
            with open(self.dir+self.pcl_name, "w") as pcl:
                for i in range(len(self.data["PCL"])):
                    pcl.write(str(self.data["PCL"][i][0]) + "," +
                              str(self.data["PCL"][i][1]) + "," +
                              str(self.data["PCL"][i][2]) + "\n")
