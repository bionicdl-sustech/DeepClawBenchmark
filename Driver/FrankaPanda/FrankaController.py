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

class FrankaController:
    def __init__(self):
        self.HOME_JOINT_VALUES = []
        self.HOME_POSE = [[0.228, 0.274, 0.25], [pi, 0, 0]]
        self.PICK_Z = 0.118
        self.PLACE_Z = 0.135
        self.calibration_tool = ''

        # ROS node initialization
        rospy.init_node('perception', anonymous=True)

        # Create move group of MoveIt for motion planning
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Panda arm setting
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        self.group.set_planner_id('RRTConnectkConfigDefault')
        self.group.set_num_planning_attempts(10)
        self.group.set_planning_time(5)
        self.group.set_max_velocity_scaling_factor(0.5)
        
        # Hand setting
        self.hand = moveit_commander.MoveGroupCommander("hand")
        self.hand.set_planner_id('RRTConnectkConfigDefault')
        

        self.tool = PoseStamped()
        self.tool.header.frame_id = "panda_link0"

        self.goHome()
        #self.open_gripper()

    def goHome(self):
        print("homing...")
        self.move(self.HOME_POSE)

    def execute(self, group, plan):
        timeout = 2
        for i in range(timeout):
            res = group.execute(plan)
            time.sleep(1)
            if not res:
                # if fail, swich robot back to mode 2 so that next move can be executed
                # https://github.com/frankaemika/franka_ros/issues/69
                os.system(
                    "rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal \"{}\"")
            else:
                break
        time.sleep(2)

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

        self.group.set_pose_target(self.tool, end_effector_link='panda_link8')
        plan = self.group.plan()
        # raw_input('wait')
        self.execute(self.group, plan)

    def movej(self,x, y, z, Rx, Ry, Rz, a=0, v=0,useJoint = False):
        if(useJoint==False):
            self.move([[x, y, z], [Rx, Ry, Rz]])
        else:
            plan = self.group.plan([x*3.14159/180.0,y*3.14159/180.0,z*3.14159/180.0,Rx*3.14159/180.0,Ry*3.14159/180.0,Rz*3.14159/180.0])
            self.execute(self.group, plan)

    def openGripper(self, distance=0.025):
        target_position = [distance, distance]
        plan = self.hand.plan(target_position)
        self.execute(self.hand, plan)

    def closeGripper(self, distance=0.005):
        target_position = [distance, distance]
        plan = self.hand.plan(target_position)
        res = self.hand.execute(plan)
        time.sleep(1)
        if not res:
            # if fail, swich robot back to mode 2 so that next move can be executed
            # https://github.com/frankaemika/franka_ros/issues/69
            os.system(
                "rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal \"{}\"")
            return 0
        return 1

    def rpy2orientation(self, row, pitch, yaw):
        q = tf.transformations.quaternion_from_euler(row, pitch, yaw, axes='sxyz')
        return q

    def verifyPosition(self, target_position):
        pass

