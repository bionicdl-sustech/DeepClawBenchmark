import os
import sys
import time

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from examples.Task import Task
from input_output.publishers.Publisher import Publisher
from input_output.observers.TimeMonitor import TimeMonitor
from input_output.observers.ImageMonitor import ImageMonitor
from input_output.observers.ResultMonitor import ResultMonitor
from modules.localization.random_seg import RandomSeg
from modules.grasp_planning.random_planner import RandomPlanner
from modules.success_label.success_label import success_label
from driver.sensors.FT.getFT_Data import detectCollision


class RandomClawMachine(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(RandomClawMachine, self).__init__(perception_system, manipulation_system, is_debug)
        time_s = time.localtime(int(time.time()))
        self.experiment_name = "experiment_" + str(time_s.tm_mon) + str(time_s.tm_mday) + \
                               str(time_s.tm_hour) + str(time_s.tm_min) + str(time_s.tm_sec)
        self.data_path = _root_path+"/data/"+os.path.basename(__file__)+"/"+self.experiment_name+"/"
        self.args = {}
        self.publisher = Publisher("publisher")
        self.time_monitor = TimeMonitor("time_monitor")
        self.image_monitor = ImageMonitor("image_monitor")
        self.result_monitor = ResultMonitor("result_monitor")
        self.publisher.registerObserver(self.time_monitor)
        self.publisher.registerObserver(self.image_monitor)
        self.publisher.registerObserver(self.result_monitor)
        self.localization_operator_pick = RandomSeg([[-0.2,0.2], [-0.76, -0.46], [0.04, 0.04]])
        self.localization_operator_place = RandomSeg([[-0.2,0.2], [-0.7, -0.5], [0.15, 0.15]])
        self.recognition_operator = ''
        self.grasp_planner = RandomPlanner([[3.14, 3.14],[0, 0],[-1.57, 1.57]])
        self.motion_planner = ''
        self.path=''
    def task_display(self):
        # initialize robots
        self.arm.set_io(8,False)
        self.arm.go_home()
        self.arm.set_io(8,False)
        #result Recorder
        self.result_monitor.dir = self.data_path
        self.result_monitor.csv_name = self.experiment_name+".csv"


        # sub-task display
        for i in range(200):
            self.args["subtask_counter"] = i

            # subtask data path
            subtask_name = "subtask_pick_" + str(self.args["subtask_counter"])
            self.path = self.data_path + subtask_name + "/"
            self.time_monitor.dir = self.path
            self.time_monitor.csv_name = self.experiment_name+".csv"
            self.image_monitor.dir = self.path

            compare_label = self.subtask_display_pick()
            print(compare_label)
            if compare_label == 1:
                sdata = {"Result": [subtask_name,compare_label]}
                self.publisher.sendData(sdata)
                self.subtask_display_place()
            if compare_label == 0:
                sdata = {"Result": [subtask_name,compare_label]}
                self.publisher.sendData(sdata)

            if compare_label == 2:
                sdata = {"Result": [subtask_name,'collision occorred!']}
                print('collision occorred!')
                raw_input("Please Continue")
                self.arm.go_home()
                self.publisher.sendData(sdata)

    def subtask_display_pick(self):
        print('subtask_display_pick')
        subtask_name = "subtask_pick_" + str(self.args["subtask_counter"])
        # segmentation
        print('segmentation')
        self.arm.set_io(8,True)
        time.sleep(3)
        get_frame_1 = self.camera.get_frame()
        self.arm.set_io(8,False)
        time.sleep(2)
        start = time.time()
        bounding_box, mask, centers = self.localization_operator_pick.display()
        end = time.time()

        tdata = {"Time": [subtask_name+' segmentation_time', end-start]}
        self.publisher.sendData(tdata)
        self.image_monitor.img_name = self.experiment_name + '_image1.png'
        idata = {"Image":get_frame_1.color_image[0]}
        self.publisher.sendData(idata)

        # recognition
        print('recognition')
        start = time.time()
        end = time.time()

        tdata = {"Time": [subtask_name+' recognition_time', end - start]}
        self.publisher.sendData(tdata)

        # grasp planning
        print('grasp planning')
        start = time.time()
        grasp_pose = self.grasp_planner.display(centers)
        end = time.time()

        tdata = {"Time": [subtask_name+' grasp_planning_time', end - start]}
        self.publisher.sendData(tdata)

        # motion planning
        print('motion planning')
        start = time.time()
        grasp_pose[2] = grasp_pose[2]+0.1
        collision_bool = self.arm.move_p(grasp_pose,2,1)
        # if collision_bool == False:
        #     self.arm.protective_release()
        #     self.arm.go_home()
        #     return 2
        grasp_pose[2] = grasp_pose[2]-0.1
        collision_bool = self.arm.move_p(grasp_pose)
        # if collision_bool == False:
        #     self.arm.protective_release()
        #     self.arm.go_home()
        #     return 2

        self.arm.set_io(8,True)
        collision_bool = self.arm.collsion_detection(3,0.1)
        # if collision_bool == False:
        #     self.arm.protective_release()
        #     self.arm.go_home()
        #     return 2

        grasp_pose[2] = grasp_pose[2]+0.1
        self.arm.move_p(grasp_pose)
        self.arm.go_home()
        end = time.time()

        tdata = {"Time": [subtask_name + ' motion_planning_time', end - start]}
        self.publisher.sendData(tdata)
        tdata = {"Time": ['grasppose', grasp_pose]}
        self.publisher.sendData(tdata)
        get_frame_2 = self.camera.get_frame()
        self.image_monitor.img_name = self.experiment_name + '_image2.png'
        idata = {"Image":get_frame_2.color_image[0]}
        self.publisher.sendData(idata)
        compare_space=[50,200,630,830]
        compare_label,img1,img2 = success_label(get_frame_1.color_image[0],get_frame_2.color_image[0],compare_space)
        self.image_monitor.img_name = self.experiment_name + '_image3.png'
        idata = {"Image":img1}
        self.publisher.sendData(idata)
        self.image_monitor.img_name = self.experiment_name + '_image4.png'
        idata = {"Image":img2}
        self.publisher.sendData(idata)

        print(compare_label)
        return compare_label

    def subtask_display_place(self):
        print('subtask_display_place')
        subtask_name = "subtask_place_" + str(self.args["subtask_counter"])

        # segmentation
        print('segmentation')
        start = time.time()
        bounding_box, mask, centers = self.localization_operator_place.display()
        end = time.time()

        tdata = {"Time": [subtask_name+' segmentation_time', end - start]}
        self.publisher.sendData(tdata)

        # recognition
        print('recognition')
        start = time.time()
        end = time.time()

        tdata = {"Time": [subtask_name+' recognition_time', end - start]}
        self.publisher.sendData(tdata)

        # grasp planning
        print('grasp planning')
        start = time.time()
        grasp_pose = self.grasp_planner.display(centers)
        end = time.time()

        tdata = {"Time": [subtask_name+' grasp_planning_time', end - start]}
        self.publisher.sendData(tdata)

        # motion planning
        print('motion planning')
        start = time.time()
        self.arm.move_p(grasp_pose)
        self.arm.set_io(8,False)
        time.sleep(2)
        self.arm.go_home()
        end = time.time()

        tdata = {"Time": [subtask_name + ' motion_planning_time', end - start]}
        self.publisher.sendData(tdata)
