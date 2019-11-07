import os
import sys
import time

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from examples.Task import Task
from input_output.publishers.Publisher import Publisher
from input_output.observers.TimeMonitor import TimeMonitor
from input_output.observers.ImageMonitor import ImageMonitor


class IOTest(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(IOTest, self).__init__(perception_system, manipulation_system, is_debug)
        self.data_path = _root_path+"/data/IOTest/experiment_1/"
        self.args = {}
        self.publisher = Publisher("publisher")
        self.time_monitor = TimeMonitor("time_monitor")
        self.image_monitor = ImageMonitor("image_monitor")
        self.publisher.registerObserver(self.time_monitor)
        self.publisher.registerObserver(self.image_monitor)

    def task_display(self):
        # results path
        self.time_monitor.dir = self.data_path
        self.time_monitor.csv_name = "experiment_1.csv"

        # sub-task display
        for i in range(10):
            self.args["subtask_counter"] = i
            self.subtask_display()



    def subtask_display(self):
        # subtask data path
        subtask_name = "subtask_" + str(self.args["subtask_counter"])
        path = self.data_path + subtask_name + "/"

        # segmentation stage
        start = time.time()
        time.sleep(100)
        end = time.time()

        tdata = {"Time", [subtask_name+' segmentation_time', end-start]}
        self.publisher.sendData(tdata)

        # recognition stage
        start = time.time()
        time.sleep(200)
        end = time.time()

        tdata = {"Time", [subtask_name+' recognition_time', end - start]}
        self.publisher.sendData(tdata)

        # pose_est stage
        start = time.time()
        time.sleep(1300)
        end = time.time()

        tdata = {"Time", [subtask_name+' pose_est_time', end - start]}
        self.publisher.sendData(tdata)

        # MOT stage
        start = time.time()
        time.sleep(400)
        end = time.time()

        tdata = {"Time", [subtask_name+' MOT_time', end - start]}
        self.publisher.sendData(tdata)
