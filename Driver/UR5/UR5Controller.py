import numpy as np
import time
import urx
from math import *
import tf
from urx.urrobot import URRobot

class UR5Controller(urx.Robot):

    def __init__(self, robot_ip='192.168.1.27'):
        self.robot_ip = robot_ip
        self.HOME_POSITION = [0.4958,-0.24967,0.00037]
        self.PlACE_JOINTPOSITION= np.array([-33.91,-108.54,-61.87,-89.68,86.80,-47.69])/180 *pi
        self.HOME_JOINTPOSITION = np.array([-78.13,-48.60,-104.10,-117.38,89.35,-75.00])/180 *pi
        self.PICK_Z=0.11
        self.PLACE_Z=0.25
        self.robot = urx.Robot(robot_ip)
        self.calibration_tool = ''
        URRobot.__init__(self, robot_ip, use_rt=False)

    def place(self):
        print("Moving to placeposition")
        self.movej(self.PlACE_JOINTPOSITION,1,1)

    def goHome(self):
        '''
        Moving to home position
        :return: None
        '''
        print("homing")
        self.movej(self.HOME_JOINTPOSITION,1,1)

    def move(self,goal_position,theta, acc=1, vel=1):
        '''
        Executing motion planning to goal pose.
        :param goal_pose: the goal pose
        :return: None
        '''
        pose = self.robot.get_pose()
        pose.set_pos(goal_position)
        pose.set_orient(tf.transformations.euler_matrix(-pi, 0, theta)[:3,:3])
        try:
            print("executing ...")
            self.robot.movex('movel', pose, acc, vel)
        except:
            print("done")
            pass

    def getPosition(self):
        pose = self.robot.get_pose()
        return pose

    def verify(self,range=0.05,acc=1,vel=1):
        pose = self.robot.get_pose()
        pose_pos = pose.get_pos()
        x=pose_pos[0]
        x1= x-range
        x2= x+range
        y=pose_pos[1]
        y1= y-range
        y2= y+range
        z= pose_pos[2]
        pose.set_pos([x1,y,z])
        self.robot.movex('movel', pose, acc, vel)
        pose.set_pos([x2,y,z])
        self.robot.movex('movel', pose, acc, vel)
        pose.set_pos([x,y,z])
        self.robot.movex('movel', pose, acc, vel)
        pose.set_pos([x,y1,z])
        self.robot.movex('movel', pose, acc, vel)
        pose.set_pos([x,y2,z])
        self.robot.movex('movel', pose, acc, vel)
        pose.set_pos([x,y,z])
        self.robot.movex('movel', pose, acc, vel)

    def openGripper(self):
        '''
        Open the gripper.
        :return: True or False
        '''
        try:
            print("Gripper Opening")
            self.robot.set_digital_out(8, 0)
            time.sleep(1)
        except:
            print("Can't open gripper, please check the state of gripper!")
            return False
        return True

    def closeGripper(self):
        '''
        Close the gripper.
        :return: True or False
        '''
        try:
            print("Gripper Closing")
            self.robot.set_digital_out(8, 1)
            time.sleep(4)
        except:
            print("Can't open gripper, please check the state of gripper!")
            return False
        return True

    def grasping(self):
        print('Grasping')
        pose = self.robot.get_pose()
        x, y = pose.pos.x, pose.pos.y
        goal_pose = ((x, y, self.PICK_Z))
        pose.set_pos(goal_pose)
        try:
            print("executing ...")
            self.robot.movex('movel', pose, acc=1, vel=1)
        except:
            print("done")
            pass
        self.close_gripper()

        pose = self.robot.get_pose()
        x, y = pose.pos.x, pose.pos.y
        goal_pose = ((x, y, self.PLACE_Z))
        pose.set_pos(goal_pose)
        try:
            print("executing ...")
            self.robot.movex('movel', pose, acc=1, vel=1)
        except:
            print("done")
            pass

    def graspJudgement(self,judgement=0.07):
        grasp_volt = self.robot.get_analog_in(2)
        sucess_label = 0
        if(grasp_volt >= judgement):
             sucess_label = 1
        else:
             sucess_label = 0
        return sucess_label,grasp_volt

    def verifyGrasp(self):
        while True:
            grasp_volt = self.robot.get_analog_in(2)
            time.sleep(0.5)
            grasp_volt_new = self.robot.get_analog_in(2)
            a = grasp_volt - grasp_volt_new
            if a > 0.01:
                continue
            else:
                pass
    def getGraspVolt(self):
        grasp_volt = self.robot.get_analog_in(2)
        return grasp_volt