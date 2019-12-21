import os
current_path = os.path.dirname(os.path.abspath(__file__))
import sys
import time
from urdf_parser_py.urdf import URDF
import socket
import numpy as np
from math import pi, cos, sin 
import PyKDL as kdl
import math
import warnings
import copy
import multiprocessing as mp

class franka():
	def __init__(self,ip,gripper=True):

		# start server
		command='gnome-terminal -e '+current_path+'/server'
		os.system(command)
		self.ip = ip
		self.num_joints = 7
		self.server_address = ('127.0.0.1', 8080)
		self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.q_home = [0, -pi/4, 0, -3 * pi/4, 0, pi/2, pi/4]
		self.robot_model = URDF.from_xml_file(current_path + "/model.urdf")
		self.max_joints = [2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973]
		self.min_joints = [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973]
		self.tree = self._kdl_tree_from_urdf_model(self.robot_model)
		self.chain = self.tree.getChain("panda_link0", "panda_link8")
		self._fk_kdl = kdl.ChainFkSolverPos_recursive(self.chain)
		self._ik_v_kdl = kdl.ChainIkSolverVel_pinv(self.chain)

		self.gripper_H = np.matrix([[ 0.7071067811865476,-0.7071067811865476, 0, 0],
									[ 0.7071067811865476, 0.7071067811865476, 0, 0],
									[ 0					, 0                 , 1,-0.1034],
									[ 0					, 0                 , 0, 1]])
		self.calibration_H = []
		
		# init
		self.init_arm(self.ip)
		self.gripper = gripper
		if self.gripper == True:
			self.init_gripper(self.ip)
		self.recover()

	def __del__(self):
		command = "kill -9 $(netstat -nlp|grep ''':8080'''|awk '{print $6}'|awk -F '[/]' '{print $1}')"
		os.system(command)
		pass
	
	def set_calibration_H(self,filename):
		self.calibration_H = np.load(filename)

	def euler_to_quat(self,r, p, y):
		sr, sp, sy = np.sin(r/2.0), np.sin(p/2.0), np.sin(y/2.0)
		cr, cp, cy = np.cos(r/2.0), np.cos(p/2.0), np.cos(y/2.0)
		return [sr*cp*cy - cr*sp*sy,
		        cr*sp*cy + sr*cp*sy,
		        cr*cp*sy - sr*sp*cy,
		        cr*cp*cy + sr*sp*sy]

	def euler_matrix(self,ai, aj, ak, axes='sxyz'):

		_AXES2TUPLE = {
		'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
		'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
		'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
		'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
		'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
		'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
		'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
		'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

		_NEXT_AXIS = [1, 2, 0, 1]

		try:
		    firstaxis, parity, repetition, frame = _AXES2TUPLE[axes]
		except (AttributeError, KeyError):
		    _ = _TUPLE2AXES[axes]
		    firstaxis, parity, repetition, frame = axes

		i = firstaxis
		j = _NEXT_AXIS[i+parity]
		k = _NEXT_AXIS[i-parity+1]

		if frame:
		    ai, ak = ak, ai
		if parity:
		    ai, aj, ak = -ai, -aj, -ak

		si, sj, sk = math.sin(ai), math.sin(aj), math.sin(ak)
		ci, cj, ck = math.cos(ai), math.cos(aj), math.cos(ak)
		cc, cs = ci*ck, ci*sk
		sc, ss = si*ck, si*sk

		M = np.identity(4)
		if repetition:
		    M[i, i] = cj
		    M[i, j] = sj*si
		    M[i, k] = sj*ci
		    M[j, i] = sj*sk
		    M[j, j] = -cj*ss+cc
		    M[j, k] = -cj*cs-sc
		    M[k, i] = -sj*ck
		    M[k, j] = cj*sc+cs
		    M[k, k] = cj*cc-ss
		else:
		    M[i, i] = cj*ck
		    M[i, j] = sj*sc-cs
		    M[i, k] = sj*cc+ss
		    M[j, i] = cj*sk
		    M[j, j] = sj*ss+cc
		    M[j, k] = sj*cs-sc
		    M[k, i] = -sj
		    M[k, j] = cj*si
		    M[k, k] = cj*ci
		return M

	def urdf_pose_to_kdl_frame(self,pose):
	    pos = [0., 0., 0.]
	    rot = [0., 0., 0.]
	    if pose is not None:
	        if pose.position is not None:
	            pos = pose.position
	        if pose.rotation is not None:
	            rot = pose.rotation
	    return kdl.Frame(kdl.Rotation.Quaternion(*self.euler_to_quat(*rot)),
	                     kdl.Vector(*pos))

	def urdf_joint_to_kdl_joint(self,jnt):
	    origin_frame = self.urdf_pose_to_kdl_frame(jnt.origin)
	    if jnt.joint_type == 'fixed':
	        return kdl.Joint(jnt.name, kdl.Joint.None)
	    axis = kdl.Vector(*jnt.axis)
	    if jnt.joint_type == 'revolute':
	        return kdl.Joint(jnt.name, origin_frame.p,
	                         origin_frame.M * axis, kdl.Joint.RotAxis)
	    if jnt.joint_type == 'continuous':
	        return kdl.Joint(jnt.name, origin_frame.p,
	                         origin_frame.M * axis, kdl.Joint.RotAxis)
	    if jnt.joint_type == 'prismatic':
	        return kdl.Joint(jnt.name, origin_frame.p,
	                         origin_frame.M * axis, kdl.Joint.TransAxis)
	    print "Unknown joint type: %s." % jnt.joint_type
	    return kdl.Joint(jnt.name, kdl.Joint.None)

	def urdf_inertial_to_kdl_rbi(self,i):
	    origin = self.urdf_pose_to_kdl_frame(i.origin)
	    rbi = kdl.RigidBodyInertia(i.mass, origin.p,
	                               kdl.RotationalInertia(i.inertia.ixx,
	                                                     i.inertia.iyy,
	                                                     i.inertia.izz,
	                                                     i.inertia.ixy,
	                                                     i.inertia.ixz,
	                                                     i.inertia.iyz))
	    return origin.M * rbi

	def _kdl_tree_from_urdf_model(self,urdf):
	    root = urdf.get_root()
	    tree = kdl.Tree(root)
	    def add_children_to_tree(parent):
	        if parent in urdf.child_map:
	            for joint, child_name in urdf.child_map[parent]:
	                child = urdf.link_map[child_name]
	                if child.inertial is not None:
	                    kdl_inert = self.urdf_inertial_to_kdl_rbi(child.inertial)
	                else:
	                    kdl_inert = kdl.RigidBodyInertia()
	                kdl_jnt = self.urdf_joint_to_kdl_joint(urdf.joint_map[joint])
	                kdl_origin = self.urdf_pose_to_kdl_frame(urdf.joint_map[joint].origin)
	                kdl_sgm = kdl.Segment(child_name, kdl_jnt,
	                                      kdl_origin, kdl_inert)
	                tree.addSegment(kdl_sgm, parent)
	                add_children_to_tree(child_name)
	    add_children_to_tree(root)
	    return tree

	def joint_kdl_to_list(self,q):return [q[i] for i in range(q.rows())]

	def joint_list_to_kdl(self,q):
	    if type(q) == np.matrix and q.shape[1] == 0:
	        q = q.T.tolist()[0]
	    q_kdl = kdl.JntArray(len(q))
	    for i, q_i in enumerate(q):
	        q_kdl[i] = q_i
	    return q_kdl

	def ik(self,pos,rot,maxiter=100):
		pos_kdl = kdl.Vector(pos[0], pos[1], pos[2])
		rot_kdl = kdl.Rotation(rot[0,0], rot[0,1], rot[0,2],
		                       rot[1,0], rot[1,1], rot[1,2],
		                       rot[2,0], rot[2,1], rot[2,2])
		frame_kdl = kdl.Frame(rot_kdl, pos_kdl)
		mins_kdl = self.joint_list_to_kdl(self.min_joints)
		maxs_kdl = self.joint_list_to_kdl(self.max_joints)
		ik_p_kdl = kdl.ChainIkSolverPos_NR_JL(self.chain, 
												mins_kdl, 
												maxs_kdl,
		                                      	self._fk_kdl, 
		                                      	self._ik_v_kdl, 
		                                      	maxiter, 
		                                      	sys.float_info.epsilon)

		# use the midpoint of the joint limits as the guess
		lower_lim = np.where(np.isfinite(self.min_joints), self.min_joints, 0.)
		upper_lim = np.where(np.isfinite(self.max_joints), self.max_joints, 0.)
		q_guess = (lower_lim + upper_lim) / 2.0
		q_guess = np.where(np.isnan(q_guess), [0.]*len(q_guess), q_guess)

		q_kdl = kdl.JntArray(self.num_joints)
		q_guess_kdl = self.joint_list_to_kdl(q_guess)

		if ik_p_kdl.CartToJnt(q_guess_kdl, frame_kdl, q_kdl) >= 0:
			res = np.array(self.joint_kdl_to_list(q_kdl))
			return res
		else:
			return None

	def send_recv_msg(self,msg):
		#print(msg)
		self.s.sendto(msg, self.server_address)
		revcData, (remoteHost, remotePort) = self.s.recvfrom(1024)
		self.revcData = revcData

	def init_arm(self,ip):
		ip = str(ip)
		self.send_recv_msg("init,"+ip)

	def init_gripper(self,ip):
		ip = str(ip)
		self.send_recv_msg("gripper_init,"+ip)
	
	def go_home(self):
		self.move_j(self.q_home)

	def move_j(self,joints,v=0.5,a=0,solve_space='J'):
		#print(_joints,type(_joints))
		if type(joints) == type(None):
			print('\n KDL fail! \n')
			return 0
		q_str = ''
		for x in joints:
			q_str = q_str +str(x)+' '
		q_str = q_str[0:-1]
		msg = 'movej,'+str(v)+':'+q_str
		#print(q_str, msg) 
		self.send_recv_msg(msg)
		return 1

	def move_p(self,position,v=0.5,a=0,solve_space='L',axes = 'sxyz'):
		rot = self.euler_matrix(position[3],position[4],position[5],axes)
		pos = np.array([position[0]/1000.0,position[1]/1000.0,position[2]/1000.0])
		#print(rot,pos)
		
		if self.gripper == True:
			rot[0,3] = pos[0]
			rot[1,3] = pos[1]
			rot[2,3] = pos[2]
			rot = rot * self.gripper_H 
			#H1 = np.hstack((rot,pos))
			#a = np.append(pos,1)
			#a = np.resize(a,(4,1))
			#b = self.gripper_H * a
			pos = np.array([rot[0,3], rot[1,3], rot[2,3]])
			#print(rot,pos)
		'''

			pos = self.gripper_H * np.vstack((pos,np.array[(1)]))
			
		'''
		q = self.ik(pos,rot)
		return self.move_j(q,v=v)

	def get_joints(self):
		self.send_recv_msg('get_joints,1')
		str_data = self.revcData
		str_data = str_data.split('\x00',1)
		str_list = str_data[0].split(' ')
		joints_list = [0]*7
		for index, x in enumerate(str_list):
			if index > 0:
				joints_list[index-1] = float(x)
		return np.array(joints_list)

	def get_pose(self):
		self.send_recv_msg('get_pose,1')
		str_data = self.revcData
		str_data = str_data.split('\x00',1)
		str_list = str_data[0].split(' ')
		pose_list = [0]*16
		for index, x in enumerate(str_list):
			if index > 0:
				pose_list[index-1] = float(x)
		pose_list = np.array(pose_list)
		pose_list.resize((4,4))
		pose = pose_list.T
		pose[0:3,3] = pose[0:3,3]*1000.0
		return pose

	def getState(self):
		pose = self.get_pose()
		joints = self.get_joints()
		return joints, pose

	def verifyState(self,name,target,error='0.2'):
		return True

	def recover(self):
		self.send_recv_msg('recover,1')

	def gripper_home(self):
		self.send_recv_msg('gripper_home,1')
	
	def switch(self,open=True):
		if open:
			self.gripper_move(78,0.1)
		else:
			self.gripper_move(0,0.1)
	def gripper_grasp(self,width,speed=0.1,force=100):
		width = width/1000.0
		msg = 'gripper_grasp,'+str(width)+':'+str(speed)+':'+str(force)
		self.send_recv_msg(msg)

	def gripper_move(self,width,speed=0.1):
		width = width/1000.0
		msg = 'gripper_move,'+str(width)+':'+str(speed)
		self.send_recv_msg(msg)

	def gripper_width(self):
		msg = 'gripper_width,1'
		self.send_recv_msg(msg)
		return self.revcData

	def is_grasp(self):
		msg = 'is_grasp,1'
		self.send_recv_msg(msg)
		return self.revcData

	def gripper_stop(self):
		msg = 'gripper_stop,1'

	#TODO
	# def real_time_control(self, control_type='q', real_time_call_back_function):
	# 	control_type ==> send UDP massage to define the control type in server
	#	real_time_back_funtion recive robot_state and duration send control data via UDP 
	
if __name__ == '__main__':

	robot = franka('192.168.1.100',gripper=True)
	robot.go_home()
