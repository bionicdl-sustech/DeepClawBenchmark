import rospy, sys, os
import moveit_commander
from geometry_msgs.msg import Pose
import time

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from Driver.Controller import Controller
from ToolKit.Configuration import *

class CobottaController(Controller):
    def __init__(self):
        super(CobottaController, self).__init__()
        self.cfg = readConfiguration('denso')
        self.HOME_POSE = self.cfg['HOME_POSE']
        self.PICK_Z = self.cfg['PICK_Z']
        self.PLACE_Z = self.cfg['PLACE_Z']
        self.pose = Pose()

        # ROS node initialization
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface',
                        anonymous=True)

        # Create move group of MoveIt for motion planning
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('arm')
        self.calibration_tool = ''
        
        # Initial
        self.goHome()
        self.openGripper()

    def goHome(self):
        print("homing...")
        self.move(self.HOME_POSE)

    def execute(self, group, plan):
        # input = raw_input("waiting for confirm...")
        res = group.execute(plan)
        time.sleep(0.1)

    def move(self, goal_pose):
        goal_position = goal_pose[0]
        goal_orientation = goal_pose[1]
        q = self.rpy2orientation(goal_orientation[0], goal_orientation[1], goal_orientation[2])
        self.group.set_start_state_to_current_state()
        self.group.clear_pose_targets()

        self.pose.position.x = goal_position[0]
        self.pose.position.y = goal_position[1]
        self.pose.position.z = goal_position[2]

        self.pose.orientation.x = q[0]
        self.pose.orientation.y = q[1]
        self.pose.orientation.z = q[2]
        self.pose.orientation.w = q[3]

        self.group.set_pose_target(self.pose)
        plan = self.group.plan()
        # raw_input('wait')
        self.execute(self.group, plan)

    def movej(self,x, y, z, Rx, Ry, Rz, a=0, v=0,useJoint = False):
        if(useJoint==False):
            self.move([[x, y, z], [Rx, Ry, Rz]])
        else:
            plan = self.group.plan([x*3.14159/180.0,y*3.14159/180.0,z*3.14159/180.0,Rx*3.14159/180.0,Ry*3.14159/180.0,Rz*3.14159/180.0])
            self.execute(self.group, plan)

    def openGripper(self):
        '''
        Open the gripper.
        :return: True or False
        '''
        try:
            os.system('sh '+ _root_path +'/Cobotta/open.sh')
        except:
            print("Can't open gripper, please check the location of open.sh!")
            return False
        return True

    def closeGripper(self):
        '''
        Open the gripper.
        :return: True or False
        '''
        try:
            os.system('sh ' +_root_path + '/Cobotta/close.sh')
        except:
            print("Can't open gripper, please check the location of close.sh!")
            return False
        return True

    def verifyPosition(self, target_position):
        pass

    def calibrating(self):
        initial_pose = self.cfg['initial_pose']
        x_step = self.cfg['x_step_length']
        y_step = self.cfg['y_step_length']
        self.move(initial_pose)
        x = initial_pose[0][0]
        y = initial_pose[0][1]
        z = initial_pose[0][2]
        for i in range(4):
            for j in range(10):
                self.move([[x+x_step*i, y+y_step*j, z], initial_pose[1]])
