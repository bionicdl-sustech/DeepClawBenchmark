from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.robots.configuration_paths.arm_configuration_path import ArmConfigurationPath
from pyrep.errors import ConfigurationError, ConfigurationPathError, IKError
from pyrep.const import ConfigurationPathAlgorithms as Algos
from typing import List, Union
import copy
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from .franka_kinematics import (FrankaKinematics,get_rotation_part,
get_transition_part,set_rotation_part,set_position_part)

class Franka(Panda):

    def __init__(self):
        super().__init__()
        self.path_point_nums = 50
        self.pose = None
        self.home_joints = [0, -np.pi/4, 0, -3 * np.pi/4, 0, np.pi/2, 0]
        self.position = self.get_position()
        self.kine = FrankaKinematics()
        self.position_min = [0.8, -0.3, 0.83]
        self.position_max = [1.0, 0.3, 1.2]
        self.gripper = PandaGripper()
        self.clear_path = False
        
    def grasp(self,env,obj:None,force_mode=False):
        '''
        gripper grasp
        '''
        while not self.gripper.actuate(0.0,0.1):
            env.step()
        self.grasped_obj = obj
        if force_mode:
            self.gripper._grasped_objects.append(self.grasped_obj)
            self.gripper._old_parents.append(self.grasped_obj.get_parent())  # type: ignore
            self.obj.set_parent(self.gripper._attach_point, keep_in_place=True)
        else:
            self.gripper.grasp(self.grasped_obj)

    def release(self,env):
        '''
        gripper open
        '''
        while not self.gripper.actuate(1.0,0.1):
            env.step()
        if self.grasped_obj is not None:
            self.gripper.release()
            self.grasped_obj = None

    def _rot_value(self,euler: Union[List[float], np.ndarray] = None,
                    quaternion: Union[List[float], np.ndarray] = None):

        if euler is not None:
            return R.from_euler('xyz',euler)
        elif quaternion is not None:
            return R.from_quat(quaternion)
        else:
            raise ValueError('input eluer or quternion')

    def _get_linear_path(self, position: Union[List[float], np.ndarray],
                        euler: Union[List[float], np.ndarray] = None,
                        quaternion: Union[List[float], np.ndarray] = None
                        ) -> ArmConfigurationPath:
        # start
        joints = self.get_joint_positions()
        H_start = self.kine.fk(joints)
        # rot ~
        rots = [get_rotation_part(H_start),self._rot_value(euler,quaternion)]
        slerp = Slerp([0,1], rots)
        times = [x/self.path_point_nums for x in range(self.path_point_nums+1)]
        interp_rots = slerp(times)
        # trans ~
        d_position = (position - self.pose)/self.path_point_nums
        # ik
        ret_floats = []
        q_guess = self.home_joints
        start_position = get_transition_part(H_start)
        for i in range(self.path_point_nums+1):
            H_target = set_rotation_part(np.eye(4),interp_rots[i])
            H_target = set_position_part(H_target,start_position)
            q = self.kine.ik(H_target, q_guess) # solve_ik
            ret_floats.append(q)
            q_guess = q
        return ArmConfigurationPath(self, ret_floats)


    def _get_nonlinear_path(self, position: Union[List[float], np.ndarray],
                            euler: Union[List[float], np.ndarray] = None,
                            quaternion: Union[List[float], np.ndarray] = None) -> ArmConfigurationPath:
        r = self._rot_value(euler,quaternion)
        H_target = set_position_part(set_rotation_part(np.eye(4),r),position)
        q_target = self.kine.ik(H_target,self.home_joints)
        #self.move_j(q_target)

    def move_j(self,q_target,env):
        _q_target = copy.copy(q_target)
        _q_target[6] += np.pi/4
        q_start = np.array(self.get_joint_positions())
        dq = (_q_target - q_start)/self.path_point_nums
        res = []
        for i in range(self.path_point_nums):
            res.append(q_start + dq * i)
        res = np.array(res)
        res = res.reshape((1,-1))
        path =  ArmConfigurationPath(self, res.tolist()[0])
        done = False
        while not done:
            done = path.step()
            env.step()

    def home(self,env):
        self.move_j(self.home_joints,env)
    
    def move(self,env,
            position: Union[List[float], np.ndarray],
            euler: Union[List[float], np.ndarray] = None,
            quaternion: Union[List[float], np.ndarray] = None):
        path = self.get_path(
            position=position, euler=euler, quaternion = quaternion)
        if path is None:
            raise RuntimeError('no path found')
        path.visualize()
        env.step()
        
        # Step the simulation and advance the agent along the path
        done = False
        while not done:
            done = path.step()
            env.step()
        if self.clear_path:
            path.clear_visualization()

    def go_to_position(self,position: Union[List[float], np.ndarray],
                            euler: Union[List[float], np.ndarray] = None,
                            quaternion: Union[List[float], np.ndarray] = None) -> ArmConfigurationPath:
        r = self._rot_value(euler,quaternion)
        H_target = set_position_part(set_rotation_part(np.eye(4),r),np.array(position))
        q = self.kine.ik(H_target,self.home_joints)
        self.set_joint_positions(q)

    def get_path(self, position: Union[List[float], np.ndarray],
                 euler: Union[List[float], np.ndarray] = None,
                 quaternion: Union[List[float], np.ndarray] = None,
                 ignore_collisions=False,
                 trials=100, max_configs=60, trials_per_goal=6,
                 algorithm=Algos.SBL
                 ) -> ArmConfigurationPath:
        '''
        para
        ---
            position(franka frame)
            euler or quaternion
        '''
        #position = np.array(position) + np.array(self.position)
        position = np.array(position)
        try:
            p = self.get_linear_path(position, euler, quaternion,
                                     ignore_collisions=ignore_collisions)
            return p
        except ConfigurationPathError:
            print('get linear path fail\n')
            pass  # Allowed. Try again, but with non-linear.
        
        try: 
            # TODO: _get_linear_path
            #p = self._get_linear_path(position,euler,quaternion)
            #return p
            pass
        except ConfigurationError:
            pass

        try:
            p = self.get_nonlinear_path(
                position, euler, quaternion, ignore_collisions, trials, max_configs,
                trials_per_goal, algorithm)
            return p
        except ConfigurationPathError:
            print('get nonlinear path fail\n')
            #p = self._get_nonlinear_path(position,euler,quaternion)
            #return p
            pass
