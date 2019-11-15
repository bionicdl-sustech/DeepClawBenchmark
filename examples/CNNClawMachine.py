import os
import sys
import time

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from examples.Task import Task
from input_output.publishers.Publisher import Publisher
from input_output.observers.TimeMonitor import TimeMonitor
from input_output.observers.ImageMonitor import ImageMonitor
from modules.end2end.grasp_alexnet.Predictor import Predictor


class CNNClawMachine(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(CNNClawMachine, self).__init__(perception_system, manipulation_system, is_debug)
        self.experiment_name = "experiment_1"
        self.data_path = _root_path+"/data/CNNClawMachine/"+self.experiment_name+"/"
        self.args = {}
        self.publisher = Publisher("publisher")
        self.time_monitor = TimeMonitor("time_monitor")
        self.image_monitor = ImageMonitor("image_monitor")
        self.publisher.registerObserver(self.time_monitor)
        self.publisher.registerObserver(self.image_monitor)

        self.network = Predictor(_root_path+"/modules/end2end/grasp_alexnet/checkpoint/xxxxx")
        self.motion_planner = ''

    def task_display(self):
        # results path
        self.time_monitor.dir = self.data_path
        self.time_monitor.csv_name = self.experiment_name+".csv"

        # sub-task display
        for i in range(10):
            self.args["subtask_counter"] = i
            self.subtask_display()



    def subtask_display(self):
        # subtask data path
        subtask_name = "subtask_" + str(self.args["subtask_counter"])
        path = self.data_path + subtask_name + "/"

        # end2end
        color_image, _ = self.camera.getImage()
        start = time.time()
        grasp_pose = self.network.predict(color_image)
        end = time.time()

        tdata = {"Time": [subtask_name+' grasp_planning_time', end - start]}
        self.publisher.sendData(tdata)

        # motion planning
        start = time.time()
        end = time.time()

        tdata = {"Time": [subtask_name+' motion_planning_time', end - start]}
        self.publisher.sendData(tdata)

        # execution
        position = [grasp_pose[0], grasp_pose[1], grasp_pose[2]]
        orientation = [grasp_pose[3], grasp_pose[4], grasp_pose[5]]
        start = time.time()
        self.arm.move([position, orientation])
        end = time.time()

        tdata = {"Time": [subtask_name + ' execution_time', end - start]}
        self.publisher.sendData(tdata)
