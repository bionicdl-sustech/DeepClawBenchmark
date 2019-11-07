import rospy
import sys
import time
from geometry_msgs.msg import Pose, PoseStamped
import moveit_commander
import os
import tf
import numpy as np

_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

from driver.arms.ArmController import ArmController
from input_output.Configuration import *


class FrankaController(ArmController):
    def __init__(self, configuration):
        super(FrankaController, self).__init__()
        self._cfg = configuration
        self._home_joints = self._cfg["HOME_JOINTS"]
        self._home_pose = self._cfg["HOME_POSE"]
        self._R = np.zeros((3, 3))
        self._t = np.zeros((3, 1))

        # Calibration
        self.matrix_load()

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

    def goHome(self):
        print("homing...")
        self.move(self._home_pose)

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

    def movej(self, x, y, z, Rx, Ry, Rz, a=0, v=0, useJoint=False):
        if useJoint == False:
            self.move([[x, y, z], [Rx, Ry, Rz]])
        else:
            plan = self.group.plan([x*3.14159/180.0, y*3.14159/180.0, z*3.14159/180.0,
                                    Rx*3.14159/180.0, Ry*3.14159/180.0, Rz*3.14159/180.0])
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

    def matrix_load(self):
        d = np.load(os.path.dirname(_root_path)+self._cfg["CALIBRATION_DIR"])
        observed_pts = d['arr_0']
        measured_pts = d['arr_1']
        self._R, self._t = self.get_rigid_transform(observed_pts, measured_pts)

    def uvd2xyz(self, u, v, depth_image, depth_scale, intrinsics):
        camera_z = np.mean(np.mean(depth_image[v - 5:v + 5, u - 5:u + 5])) * depth_scale
        camera_x = np.multiply(u - intrinsics.ppx, camera_z / intrinsics.fx)
        camera_y = np.multiply(v - intrinsics.ppy, camera_z / intrinsics.fy)

        view = depth_image[v - 30:v + 30, u - 30:u + 30]
        view[view == 0] = 10000
        avoid_z = np.min(view)
        avoid_v = np.where(depth_image[v - 30:v + 30, u - 30:u + 30] == avoid_z)[0][0] + v - 5
        avoid_u = np.where(depth_image[v - 30:v + 30, u - 30:u + 30] == avoid_z)[1][0] + u - 5
        avoid_x = np.multiply(avoid_u - intrinsics.ppx, avoid_z * depth_scale / intrinsics.fx)
        avoid_y = np.multiply(avoid_v - intrinsics.ppy, avoid_z * depth_scale / intrinsics.fy)

        xyz = self._R.dot(np.array([camera_x, camera_y, camera_z]).T) + self._t.T
        avoid_xyz = self._R.dot(np.array([avoid_x, avoid_y, avoid_z * depth_scale]).T) + self._t.T
        return list(xyz.T), avoid_xyz[2]

    def calibrating3d(self, camera):
        from modules.calibration.Calibration3D import image_callback
        calibration_cfg = readConfiguration(_root_path+"/config/modules/calibration3d.yaml")
        initial_pose = calibration_cfg['initial_pose']
        x_step = calibration_cfg['x_step_length']
        y_step = calibration_cfg['y_step_length']
        z_step = calibration_cfg['z_step_length']

        self.move(initial_pose)
        x = initial_pose[0][0]
        y = initial_pose[0][1]
        z = initial_pose[0][2]

        observed_pts = []
        measured_pts = []
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    self.move([[x + x_step * i, y + y_step * j, z + z_step * k], initial_pose[1]])
                    color_image, info = camera.getImage()
                    depth_image = info[0]
                    observed_pt = image_callback(color_image, depth_image,
                                                 camera.get_depth_scale(), camera.get_intrinsics())
                    measured_pt = [x + x_step * i, y + y_step * j, z + z_step * k + 0.17]
                    if len(observed_pt) != 0:
                        observed_pts.append(observed_pt)
                        measured_pts.append(measured_pt)
        np.savez(os.path.dirname(_root_path)+self._cfg["CALIBRATION_DIR"], observed_pts, measured_pts)
