import rospy
import sys
import time
from geometry_msgs.msg import Pose, PoseStamped
import moveit_commander
import os
import tf
from math import pi

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_path)

from Driver.Aubo.io_interface import *

class AuboController:
    def __init__(self):
        self.HOME_JOINT_VALUES = [-80.25 / 180 * 3.14, 2.0 / 180 * 3.14, 125.8 / 180 * 3.14, 34.1 / 180 * 3.14,
                                   96.1 / 180 * 3.14, 12.7 / 180 * 3.14]
        self.HOME_POSE = [[0.28, 0.1, 0.3], [pi, 0, 0]]
        self.PICK_Z = 0.115
        self.PLACE_Z = 0.12
        self.calibration_tool = ''

        # ROS node initialization
        rospy.init_node('perception', anonymous=True)

        # Create move group of MoveIt for motion planning
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Panda arm setting
        self.group = moveit_commander.MoveGroupCommander("manipulator")

        self.group.set_planner_id('RRTConnectkConfigDefault')
        self.group.set_num_planning_attempts(5)
        self.group.set_planning_time(5)
        self.group.set_max_velocity_scaling_factor(0.5)

        self.tool = PoseStamped()
        self.tool.header.frame_id = 'base_link'
        # Hand setting

        self.goHome()
        set_states()
        #self.open_gripper()

    def goHome(self, useJoint=False):
        print("homing...")
        if useJoint==False:
            self.move(self.HOME_POSE)
        else:
            self.movej(self.HOME_JOINT_VALUES[0], self.HOME_JOINT_VALUES[1], self.HOME_JOINT_VALUES[2],
                       self.HOME_JOINT_VALUES[3], self.HOME_JOINT_VALUES[4], self.HOME_JOINT_VALUES[5])

    def execute(self, group, plan):
        timeout = 2
        # raw_input('wait')
        for i in range(timeout):
            res = group.execute(plan)
            #time.sleep(1)
        # time.sleep(2)

    def move(self, goal_pose):
        goal_position = goal_pose[0]
        goal_orientation = goal_pose[1]
        q = self.rpy2orientation(goal_orientation[0], goal_orientation[1], goal_orientation[2])
        self.group.set_start_state_to_current_state()
        self.group.clear_pose_targets()

        self.tool.pose.position.x = goal_position[0]
        self.tool.pose.position.y = goal_position[1]
        self.tool.pose.position.z = goal_position[2]

        self.tool.pose.orientation.x = q[0]
        self.tool.pose.orientation.y = q[1]
        self.tool.pose.orientation.z = q[2]
        self.tool.pose.orientation.w = q[3]

        self.group.set_pose_target(self.tool, end_effector_link='wrist3_Link')
        plan = self.group.plan()
        self.execute(self.group, plan)

    def movej(self,x, y, z, Rx, Ry, Rz, a=0, v=0, useJoint=False):
        if useJoint == False:
            self.move([[x, y, z], [Rx, Ry, Rz]])
        else:
            self.group.set_joint_value_target([x, y, z, Rx, Ry, Rz])
            plan = self.group.plan()
            self.execute(self.group, plan)


    def openGripper(self):
        set_digital_out(0, False)

    def closeGripper(self):
        set_digital_out(0, True)

    def rpy2orientation(self, row, pitch, yaw):
        q = tf.transformations.quaternion_from_euler(row, pitch, yaw, axes='sxyz')
        return q

    def verifyPosition(self, target_position):
        pass

