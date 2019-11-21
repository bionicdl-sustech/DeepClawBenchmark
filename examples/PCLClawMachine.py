import os
import sys
import time

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from examples.Task import Task
from input_output.publishers.Publisher import Publisher
from input_output.observers.TimeMonitor import TimeMonitor
from input_output.observers.PCLMonitor import PCLMonitor
from modules.localization.dbscan import dbscan
from modules.grasp_planning.principal_axis_planner import PrincipalAxisPlanner


class PCLClawMachine(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(PCLClawMachine, self).__init__(perception_system, manipulation_system, is_debug)
        self.experiment_name = "experiment_1"
        self.data_path = _root_path+"/data/PCLClawMachine/"+self.experiment_name+"/"
        self.args = {}
        self.publisher = Publisher("publisher")
        self.time_monitor = TimeMonitor("time_monitor")
        self.pcl_monitor = PCLMonitor("pcl_monitor")
        self.publisher.registerObserver(self.time_monitor)
        self.publisher.registerObserver(self.pcl_monitor)

        self.localization_operator = dbscan(0.5, 5)
        self.recognition_operator = ''
        self.grasp_planner = PrincipalAxisPlanner()
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
        self.pcl_monitor.dir = path + "pcl/"
        self.pcl_monitor.pcl_name = subtask_name + ".pcl"

        # segmentation
        point_cloud = self.camera.getPointCloud()

        start = time.time()
        boundingbox, mask, centers = self.localization_operator.display(point_cloud=point_cloud)
        end = time.time()

        tdata = {"Time": [subtask_name+' segmentation_time', end-start]}
        pdata = {"PCL": point_cloud}
        self.publisher.sendData(tdata)
        self.publisher.sendData(pdata)

        # recognition
        start = time.time()
        end = time.time()

        tdata = {"Time": [subtask_name+' recognition_time', end - start]}
        self.publisher.sendData(tdata)

        # grasp planning
        start = time.time()
        grasp_pose = self.grasp_planner.display(point_cloud, centers, mask)
        end = time.time()

        tdata = {"Time": [subtask_name+' grasp_planning_time', end - start]}
        self.publisher.sendData(tdata)

        # motion planning
        start = time.time()
        end = time.time()

        tdata = {"Time": [subtask_name+' motion_planning_time', end - start]}
        self.publisher.sendData(tdata)

        # execution
        start = time.time()
        self.arm.move_p(grasp_pose)
        end = time.time()

        tdata = {"Time": [subtask_name + ' execution_time', end - start]}
        self.publisher.sendData(tdata)
