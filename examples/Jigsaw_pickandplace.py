import cv2
import os
import sys
import time
import copy

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from examples.Task import Task
from input_output.publishers.Publisher import Publisher
from input_output.observers.TimeMonitor import TimeMonitor
from input_output.observers.ImageMonitor import ImageMonitor


from input_output.observers.JigsawDataMonitor import JigsawDataMonitor
from modules.localization.contour_filter import contour_filter
from modules.localization.DetectForeground import Segment
from modules.end2end.SSD.seg_rec import seg_rec
from modules.grasp_planning.AlexNet.PredictAngleWithAlex import PredictAngle
from modules.calibration.Calibration2D import calibration
from scipy.spatial.transform import Rotation as R


class Jigsaw(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(Jigsaw, self).__init__(perception_system, manipulation_system, is_debug)
        time_s = time.localtime(int(time.time()))
        self.experiment_name = "experiment_" + str(time_s.tm_mon) + str(time_s.tm_mday) + \
                               str(time_s.tm_hour) + str(time_s.tm_min) + str(time_s.tm_sec)
        self.data_path = _root_path+"/data/"+os.path.basename(__file__).split(".")[0]+"/"+self.experiment_name+"/"
        # ymin,ymax,xmin,xmax
        self.args = {"WorkSpace": [[350, 550], [450, 850]]}
        self.publisher = Publisher("publisher")
        self.time_monitor = TimeMonitor("time_monitor")
        self.image_monitor = ImageMonitor("image_monitor")
        self.data_monitor = JigsawDataMonitor("data_monitor")
        self.publisher.registerObserver(self.time_monitor)
        self.publisher.registerObserver(self.image_monitor)
        self.publisher.registerObserver(self.data_monitor)

        # self.localization_operator = contour_filter(area_threshold=[300, 500])
        # self.recognition_operator = color_recognition([100, 43, 46], [124, 255, 255])
        self.grasp_planner = self.PickPlan
        self.motion_planner = ''

    def task_display(self):
        # results path
        self.time_monitor.dir = self.data_path
        self.time_monitor.csv_name = self.experiment_name+"_CostTime.csv"
        self.data_monitor.dir = self.data_path
        self.data_monitor.csv_name = self.experiment_name+"_JigsawData.csv"

        # task begin
        # in this task, we only take one picture and complete the task.
        # the tray area [120:650,470:1150] ymin:ymax,min:xmax
        # home Joint [-65.29,-56.93,-139.39,-73.65,89.48,24.57]
        # home position  [0.00,-262.68,251.71,3.1328,0.0131,0.0110]
        works_box = [470,120,850,650] # [xmin,ymin,xmax,ymax] , in image coordinate
        place_box = [850,120,1150,650] # [xmin,ymin,xmax,ymax] , in image coordinate

        place_position = [246.20, -533.9, 93.0, 3.14, 0.0, 0.0]
        # gohome
        self.arm.goHome()
        # ==================================== task begin ===================================================================
        # take a picture without objects
        frame= self.camera.get_frame()
        color_image_background = frame.color_image[0]
        #save the original picture
        cidata = {"Image": color_image_background}
        self.image_monitor.img_name = "background_image.jpg"
        self.publisher.sendData(cidata)


        # place the pieces, start the task
        raw_input(' task begin\n press Enter:')
        # take a picture when objects are placed
        frame = self.camera.get_frame()
        color_image = frame.color_image[0]

        cidata = {"Image": color_image}
        self.image_monitor.img_name = "initial_image.jpg"
        self.publisher.sendData(cidata)

        # ==================================== localization ground truth =====================================================
        #the groundTruth of object rect
        rect_truth = Segment(works_box).DiffGround(color_image_background,color_image)
        showed_image = color_image.copy()
        for i in range(len(rect_truth)):
            cv2.rectangle(showed_image,(int(rect_truth[i][0]),int(rect_truth[i][1])),(int(rect_truth[i][2]),int(rect_truth[i][3])),(0,255,0),2)

        cidata = {"Image": showed_image}
        self.image_monitor.img_name = "groundTruth_image.jpg"
        self.publisher.sendData(cidata)

        if True:
            cv2.imshow('groundTrue',showed_image)
            cv2.waitKey()
        # ==================================== localization and recognition =====================================================
        # segmentation and recognition
        start = time.time()
        sr = seg_rec(works_box)
        sr.setCheckPoint(_root_path + '/modules/end2end/SSD/SSD-Tensorflow/checkpoints/model.ckpt-454522')
        rclasses rscores, rect_seg = sr.display(color_image)
        end = time.time()
        tdata = {"Time": ['localization&recognition', end - start]}
        self.publisher.sendData(tdata)

        # save prediction image
        showed_image = color_image.copy()
        for i in range(len(rect_seg)):
            cv2.rectangle(showed_image,(int(rect_seg[i][0]),int(rect_seg[i][1])),(int(rect_seg[i][2]),int(rect_seg[i][3])),(0,255,0),2)
            cv2.putText(showed_image,'label: %d'%rclasses[i], org=(int(rect_seg[i][0]),int(rect_seg[i][1])), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.5, color=(0, 0, 255), thickness=2)
        cidata = {"Image": showed_image}
        self.image_monitor.img_name = "prediction_image.jpg"
        self.publisher.sendData(cidata)


        # ==================================== pick planning =====================================================
        # pick position and angle
        pick_pose = [] # [[u,v,angle],...]
        pick_class = [] # [class,...]
        pick_pose_base = []
        hand_eye = calibration()
        for i in range(len(rect_seg)):
            if(rclasses[i] > 4):
                continue
            start = time.time()
            temp = self.grasp_planner(rect_seg[i],color_image)
            pick_pose.append(temp)
            pick_class.append(rclasses[i])
            # calibration,covert to robot base
            x_p,y_p = hand_eye.cvt(temp[0],temp[1])
            pick_pose_base.append([x_p,y_p,temp[2]])

            end = time.time()
            tdata = {"Time": ['grasp_planner_'+str(rclasses[i]), end - start]}
            self.publisher.sendData(tdata)
        # show the pick points in the image
        showed_image = color_image.copy()
        for i in range(len(pick_pose)):
            cv2.circle(showed_image, (int(pick_pose[i][0]), int(pick_pose[i][1])), 5, (255,0,0), 4)
            cv2.putText(showed_image,'a:%.2f'%(pick_pose[i][2]*180/3.14),(int(pick_pose[i][0]), int(pick_pose[i][1])),cv2.FONT_HERSHEY_SIMPLEX,1, (209, 80, 0, 255), 2)

        if(self.is_debug):
            cv2.imshow('pickPoint',showed_image)
            cv2.waitKey()

        cidata = {"Image": showed_image}
        self.image_monitor.img_name = "pick_pose_image.jpg"
        self.publisher.sendData(cidata)
        # ==================================== place planning =====================================================
        # placed pose (x,y,angle) in robot base
        start = time.time()
        place_pos = []
        place_pos = self.PlacePlan(color_image,place_box)
        end = time.time()
        tdata = {"Time": ['place_planner'), end - start]}
        self.publisher.sendData(tdata)

        # ==================================== execution =====================================================
        # corresponding pick and place position
        for i in range (len(rclass)):
            place_pos.append(place_pose[rclass[i]-1])

        for i in range(len(pick_pos)):
            start = time.time()
            subtask_display(pick_pose_base[i],pick_class[i],place_pos[i])
            end = time.time()
            tdata = {"Time": ['pick_execution_'+str(i), end - start]}
            self.publisher.sendData(tdata)

        #score
        score = Result()
        cv2.putText(color_image,'Task score: ' + task_result, org=(10, 200), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.8, color=(0, 0, 255), thickness=2)
        #save final result
        frame = self.camera.get_frame()
        color_image = frame.color_image[0]
        cidata = {"Image": color_image}
        self.image_monitor.img_name = "result.jpg"
        self.publisher.sendData(cidata)


        finaldata = {"JigsawData": [rect_truth, rect_seg, rclasses, pick_pose,place_pos,score]}
        self.publisher.sendData(finaldata)

    # for pick and place, and assembly
    def Result():
        str_complete = raw_input("Enter the scores nmu: ")
        sc = float(str_complete)
        return sc

    #placed on the template
    # the place position is different in different tasks
    def PlacePlan(self,color_image,place_box):
        # in the place area
        # the black block
        crop_img = color_image[place_box[1]:place_box[3],place_box[0]:place_box[2]]
        lower = np.array([120, 120, 120])
        upper = np.array([255, 255, 255])
        mask = cv2.inRange(crop_img, lower, upper)

        img_medianBlur=cv2.medianBlur(mask,5)
        mask = cv2.bitwise_not(img_medianBlur)

        if self.is_debug:
            cv2.imshow('mask',mask)
            cv2.waitKey()

        result = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if(len(result)==2):
            contours = result[0]
            hierarchy = result[1]
        elif(len(result)==3):
            contours = result[1]
            hierarchy = result[2]

        placed_pose = []
        placed_in_image= []
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area < 20*20 or area > 200*200:
                continue
            else:
                min_Box = cv2.minAreaRect(contours[i])
                box = cv2.boxPoints(min_Box)
                box = np.intp(box)
                cv2.drawContours(crop_img, [box], -1, (255,0,0),2)
                if self.is_debug:
                    cv2.imshow('contours',crop_img)
                    cv2.waitKey(1)
                x,y = min_Box[0]
                x = x + place_box[0]
                y = y + place_box[1]
                # show the pick points in the image
                cv2.circle(color_image, (int(x), int(y)), 5, (255,0,0), 4)
                if(self.is_debug):
                    cv2.imshow('pickPoint',color_image)
                    cv2.waitKey()

                # calibration
                hand_eye = calibration()
                x_p,y_p = hand_eye.cvt(x,y)
                angle = min_Box[2]
                # return
                placed_pose.append([x_p,y_p,angle*3.14/180.0])
                placed_in_image.append([x,y,angle*3.14/180.0])
        return placed_pose


    def subtask_display(self,pick_pos,pick_class,place_pos):
        up_z = 40 #mm
        pick_z = 78
        place_z = 85
        init_rpy = [3.14,0,-0.0*3.14/180.0]
        pick_rpy = init_rpy
        pick_rpy[2] = init_rpy[2]-pick_pos[2]

        # euler to rotationVector
        r = R.from_euler('xyz', [pick_rpy[0],pick_rpy[1],pick_rpy[2]], degrees=False)
        Rx, Ry, Rz = r.as_rotvec()

        # go above the pick position
        pick_up_pos =[pick_pos[0], pick_pos[1], pick_z+up_z, Rx, Ry, Rz]
        self.arm.movep(pick_up_pos, 1.2, 1.6,False)

        # go down and pick
        pick_pos = [pick_pos[0],pick_pos[1],pick_z,Rx, Ry, Rz]
        self.arm.movep(pick_pos, 1.2, 1.6)

        # end-effector action
        suction_on()
        self.suction_action(0,False)
        time.sleep(1.5)
        # go above the pick position
        pick_up_pos = [pick_pos[0],pick_pos[1],pick_z+up_z,Rx, Ry, Rz]
        self.arm.movep(pick_up_pos, 1.2, 1.6,False)

        # go above the place position
        place_rpy = [3.14,0,-0.0*3.14/180.0]
        place_rpy[2] = place_rpy[2] - place_pos[2]
        # rv_place = rpy2rotation(place_rpy[0],place_rpy[1],place_rpy[2])
        # euler to rotationVector
        r = R.from_euler('xyz', [place_rpy[0],place_rpy[1],place_rpy[2]], degrees=False)
        Rx, Ry, Rz = r.as_rotvec()

        place_up_pos = [place_pos[0],place_pos[1],place_z+up_z,Rx, Ry, Rz]
        self.arm.movep(place_up_pos, 1.2, 1.6,False)
        # go down and place
        place_pos = [place_pos[0],place_pos[1],place_z,Rx, Ry, Rz]
        self.arm.movep(place_pos, 0.8, 0.8,False)

        # end-effector release
        self.suction_action(0,True)
        time.sleep(1.5)

        # go above the place position
        place_up_pos = [place_pos[0],place_pos[1],place_z+up_z,Rx, Ry, Rz]
        self.arm.movep(place_up_pos, 1.2, 1.6,False)

        # go home
        self.arm.goHome()

    def suction_action(self,input=0,state=True):
        self.robot_gripper.io_controller(input,state)

    def PickPlan(self,rect,color_image):
        xmin = rect[0]
        ymin = rect[1]
        xmax = rect[2]
        ymax = rect[3]

        crop_img = color_image[ymin-10:ymax+10,xmin-10:xmax+10]
        pp = PredictAngle()
        pp.loadImg(crop_img)
        angle = pp.display()

        x = (xmin+xmax)/2.0
        y = (ymin+ymax)/2.0
        return x,y,angle
