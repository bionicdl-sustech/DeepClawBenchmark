import os
import sys
_root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(_root_path)

import cv2
import time
import copy
import numpy as np
import multiprocessing as mp
from math import pi, cos, sin
import random
import cv2.aruco as aruco
from scipy.spatial.transform import Rotation as R


from models.end2end.grasp_alexnet.fc_Predictor import Predictor as G
from examples.Task import Task


class Board(object):
    def __init__(self):
        self._board = ['-' for _ in range(9)]
        self._history = []

    def _move(self, action, take):
        if self._board[action] == '-':
            self._board[action] = take

            self._history.append((action, take))

    def _unmove(self, action):
        self._board[action] = '-'

        self._history.pop()

    def get_board_snapshot(self):
        return self._board[:]

    def get_legal_actions(self):
        actions = []
        for i in range(9):
            if self._board[i] == '-':
                actions.append(i)
        return actions

    def check_avalible_action(self):
        actions = self.get_legal_actions()
        return bool(len(actions))

    def is_legal_action(self, action):
        return self._board[action] == '-'

    def teminate(self):
        board = self._board
        lines = [board[0:3], board[3:6], board[6:9], board[0::3], board[1::3], board[2::3], board[0::4], board[2:7:2]]

        if ['X'] * 3 in lines or ['O'] * 3 in lines or '-' not in board:
            return True
        else:
            return False

    def get_winner(self):
        board = self._board
        lines = [board[0:3], board[3:6], board[6:9], board[0::3], board[1::3], board[2::3], board[0::4], board[2:7:2]]

        if ['X'] * 3 in lines:
            return 0
        elif ['O'] * 3 in lines:
            return 1
        else:
            return 2

    def print_history(self):
        print(self._history)

    def set_borad(self, m):
        m = np.array(m)
        m = m.flatten()
        board = []
        for num in m:
            if num == 0:
                board.append('-')
            elif num == 1:
                board.append('O')
            elif num == 2:
                board.append('X')
        self._board = board

    def print_board(self):
        for i in range(3):
            j = i*3
            line = self._board[j:j+3]
            print(' '*3 + '| ' + line[0] + ' ' +line[1] + ' ' +line[2] + ' |')

class tictactoe_ai():

    def __init__(self, take='X'):
        self.take = take

    def think(self, board):
        #print('\rThinking...')
        take = ['X', 'O'][self.take == 'X']
        player = tictactoe_ai(take)
        _, action = self.minimax(board, player)
        #print('\rFind solution!')
        return action

    def minimax(self, board, player, depth=0):
        if self.take == "O":
            bestVal = -10
        else:
            bestVal = 10

        if board.teminate():
            if board.get_winner() == 0:
                return -10 + depth, None
            elif board.get_winner() == 1:
                return 10 - depth, None
            elif board.get_winner() == 2:
                return 0, None

        for action in board.get_legal_actions():
            board._move(action, self.take)
            val, _ = player.minimax(board, self, depth + 1)
            board._unmove(action)

            if self.take == "O":
                if val > bestVal:
                    bestVal, bestAction = val, action
            else:
                if val < bestVal:
                    bestVal, bestAction = val, action

        return bestVal, bestAction

class TechDemo(Task):
    def __init__(self, perception_system, manipulation_system, is_debug=False):
        super(TechDemo, self).__init__(perception_system, manipulation_system, is_debug)

    def realsense_run(self,qr_location,q):
		fx, fy, ppx, ppy,_ = self.camera.getIntrinsics()
		colortonum = {'yellow':0, 'green':1, 'red':2, 'blue':3, 'purple':4, 'orange':5,'unknow':6}
		mtx = np.array([[fx, 0, ppx],
		                [0,  fy, ppy],
		                [0,  0,  1]])
		dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
		lower_y = np.array([17, 100, 100])
		upper_y = np.array([35, 255, 255])

		lower_o = np.array([5, 100, 100])
		upper_o = np.array([16, 255, 255])

		lower_p = np.array([130, 50, 10])
		upper_p = np.array([170, 150, 110])

		lower_b = np.array([100, 100, 30])
		upper_b = np.array([130, 255, 130])

		lower_r = np.array([0, 100, 80])
		upper_r = np.array([5, 255, 255])

		lower_g = np.array([40, 40, 10])
		upper_g = np.array([70, 255, 115])

		aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
		parameters =  aruco.DetectorParameters_create()
		def aruco_marker_callback(color_img):
			
			gray_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
			corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img, aruco_dict, parameters=parameters)
			if ids is None:
				return 0,0,0,0
			rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.015, mtx, dist)

			return rvec,tvec,ids,corners

		while True:
			frame = self.camera.get_frame()
        	realsense_img = frame.color_image[0]
			edited_img = copy.copy(realsense_img)
			rvec,tvec,ids,corners = aruco_marker_callback(realsense_img)

			avg_colors=[]
			colors =[]
			corners = np.array(corners)
			if type(rvec) == int:
				q.put((realsense_img,edited_img,rvec,tvec,ids,avg_colors,colors))
				continue
			for i in range(rvec.shape[0]):
				aruco.drawDetectedMarkers(edited_img, corners)
				aruco.drawAxis(edited_img, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.01)
			for i in range(rvec.shape[0]):
				loc = np.sum(corners[i,0],axis=0)/4
				pts1 = np.float32([corners[i,0,3],corners[i,0,0],corners[i,0,2],corners[i,0,1]])
				pts2 = np.float32([[100,100],[260,100],[100,260],[260,260]])
				M = cv2.getPerspectiveTransform(pts1,pts2)
				dst = cv2.warpPerspective(edited_img,M,(350,350))
				avg_color = np.sum([dst[60,60],dst[300,60],dst[60,300],dst[300,300]],axis=0)/4 # (1,1,3)
				avg_colors.append(avg_color)
				avg_color = np.array([[avg_color]],dtype=np.uint8)
				hsv = cv2.cvtColor(avg_color, cv2.COLOR_BGR2HSV)
				# print(hsv)
				if cv2.inRange(hsv, lower_y, upper_y)[0,0] == 255:
				    color = 'yellow'
				elif cv2.inRange(hsv, lower_g, upper_g)[0,0] == 255:
				    color = 'green'
				elif cv2.inRange(hsv, lower_r, upper_r)[0,0] == 255:
				    color = 'red'
				elif cv2.inRange(hsv, lower_b, upper_b)[0,0] == 255:
				    color = 'blue'
				elif cv2.inRange(hsv, lower_p, upper_p)[0,0] == 255:
				    color = 'purple'
				elif cv2.inRange(hsv, lower_o, upper_o)[0,0] == 255:
				    color = 'orange'
				else:
				    color = 'unknow'
				    # print(hsv)
				colors.append(color)
				cv2.putText(edited_img," "+str(ids[i,0])+color,(int(loc[0]),int(loc[1])),cv2.FONT_HERSHEY_SIMPLEX,
				    1, (int(avg_color[0,0,0]),int(avg_color[0,0,1]),int(avg_color[0,0,2])),2,cv2.LINE_AA)
			if False:
				os.system('clear')
				print('\t'+'ID'+'\t'+'color'+'\t'*2+'R'+'\t'*2+'T')
				print('\t'+'_ '*30)
				print(' ')
				color_ids = []
				my_id = []
				r_i = []

				for i in range(rvec.shape[0]):
				    r_i.append(i)
				    color_ids.append(colortonum[colors[i]])
				    my_id.append(ids[i,0])
				id_all = np.array(color_ids) + np.array(my_id)*7
				r_i = [r_i[i] for i in np.argsort(id_all)]
				for i in r_i:
				    #print_list(ids[i,0],colors[i],rvec[i,0,:],tvec[i,0,:])
				    (' ' if rvec[i,0,0] > 0.0 else '')
				    print('\t'+str(ids[i,0])+'\t'+colors[i]+'\t'*2+(' ' if rvec[i,0,0] > 0.0 else '')+str(round(rvec[i,0,0],3))\
				        +'\t'*2+(' ' if tvec[i,0,0] > 0.0 else '')+str(round(tvec[i,0,0],3)))

				    print('\t'*4+(' ' if rvec[i,0,1] > 0.0 else '')+str(round(rvec[i,0,1],3))\
				        +'\t'*2+(' ' if tvec[i,0,1] > 0.0 else '')+str(round(tvec[i,0,1],3)))

				    print('\t'*4+(' ' if rvec[i,0,2] > 0.0 else '')+str(round(rvec[i,0,2],3))\
				        +'\t'*2+(' ' if tvec[i,0,2] > 0.0 else '')+str(round(tvec[i,0,2],3)))
				    
				    print('\t'+'_ '*30)
				    print(' ')
			q.put((realsense_img,edited_img,rvec,tvec,ids,avg_colors,colors))
			# q.put((realsense_img,ids,colors))

	def show_qr(self,q):
		while True:
			_,color_image,_,_,_,_,_ = q.get()
			cv2.imshow('img',color_image)
			if cv2.waitKey(1)==ord('q'):
				break

	def show_slides(q,which_slide,img_show_control,img_location,show_num):
		cv2.namedWindow('img', cv2.WINDOW_NORMAL)
		cv2.setWindowProperty('img', 
		        cv2.WND_PROP_FULLSCREEN, 
		        cv2.WINDOW_FULLSCREEN)
		imgs = []
		for i in range(1,16):
			imgs = imgs + [cv2.imread('./imgs/Slide'+str(i)+'.png')]
		while True:
			img = copy.copy(imgs[which_slide.value])

			if img_show_control.value != 0:
				color_image,_,_,_,_,_,_ = q.get()
				y,x = color_image.shape[:2]
				#color_image = cv2.resize(color_image,(x/2,y/2), interpolation=cv2.INTER_CUBIC)
				lx = img_location[0]
				ly = img_location[1]
				iy,ix = color_image.shape[:2]
				img[ly:iy+ly,lx:ix+lx] = color_image
			if show_num.value > 0:
				cv2.putText(img,str(show_num.value),(300,200),cv2.FONT_HERSHEY_SIMPLEX,
				    1, (255,255,255),2,cv2.LINE_AA)
			cv2.imshow('img',img)
			if cv2.waitKey(1)==ord('q'):
				break

	def s1(which_slide,img_show_control,img_location):
		robot = self.arm
		# dance 4 times!
		home = copy.copy(robot.q_home)
		for _ in range(4):
			temp = [i+(pi/8)*(random.random()-0.5) for i in home]
			robot.(temp)
		robot.move_j(robot.q_home)
		
		which_slide.value = 1
		robot.gripper_move(0)
		robot.move_p([320,0,370,pi,pi/16,0],axes = 'sxyz')
		joints = robot.get_joints()
		joints[3] = joints[3] - pi/16
		robot.move_j(joints)
		joints[3] = joints[3] + pi/32
		robot.move_j(joints)
		joints[3] = joints[3] - pi/32
		robot.move_j(joints)
		#img_show_control.value = 1
		#time.sleep(5)
		#img_show_control.value = 0
		time.sleep(0.5)
		which_slide.value = 2
		time.sleep(1)
		robot.move_j(robot.q_home)
	def s2(q,which_slide): # grasp
		robot = self.arm
		which_slide.value = 3
		robot.gripper_move(50)
		l_cube = [[[465,-65,40,pi,0,0],[465,-65,0,pi,0,0],[465,35,40,pi,0,0],[465,35,0,pi,0,0]],
				  [[500,-65,40,pi,0,0],[500,-65,0,pi,0,0],[500,35,40,pi,0,0],[500,35,0,pi,0,0]],
				  [[535,-65,40,pi,0,0],[535,-65,0,pi,0,0],[535,35,40,pi,0,0],[535,35,0,pi,0,0]]]
	  	# wait qr code
	  	qr = 0
	  	while qr <=10:
	  		_,_,_,_,ids,_,colors = q.get()
	  		if type(ids) != int:
	  			for i in range(ids.shape[0]):
	  				if ids[i,0] == 0 and colors[i] == 'yellow':
	  					qr = qr + 1
		self.robot_say_ok()
	  	for i in range(3):
	  		# get
	  		robot.gripper_move(50)
	  		robot.move_p(l_cube[i][0])
	  		robot.move_p(l_cube[i][1])
	  		robot.gripper_grasp(20)
	  		robot.move_p(l_cube[i][0])
	  		# put
	  		robot.move_p(l_cube[i][2])
	  		robot.move_p(l_cube[i][3])
	  		robot.gripper_move(50)
	  		robot.move_p(l_cube[i][2])
		robot.moveJ(robot.q_home)
		
		#robot.moveJ(robot.q_home)
	def s3(q,which_slide): # calibration
		robot = self.arm
		which_slide.value = 4
		measured_points = []
		observed_points = []
		robot.move_j(robot.q_home)
		
		for _ in range(10):
			_,_,_,_,ids,_,colors = q.get()
		qr = 0
	  	while qr <=10:
	  		_,_,_,_,ids,_,colors = q.get()
	  		if type(ids) != int:

	  			for i in range(ids.shape[0]):
	  				if ids[i,0] == 0 and colors[i] == 'yellow':
	  					
	  					qr = qr + 1
		self.robot_say_ok()

		robot.move_p([500,0,0,pi,0,0],axes = 'sxyz')
		robot.gripper_move(50)
		robot.gripper_grasp(10,0.1,100)
		robot.move_p([500,0,100,pi,0,0],axes = 'sxyz')

		which_slide.value = 5
		for x in range(3):
			x = x * 40 + 450
			for y in range(5):
				y = y * 50 - 100
				for z in range(3):
					z = z * 40 + 10
					run_good = robot.move_p([x,y,z,pi,0,0],axes='sxyz')
					if run_good:
						T = np.array([[-15],[0],[10.5],[1.0]])
						pose = robot.get_pose() * T.T
						measured_points.append(pose[0:4,3])
						while True:
							time.sleep(0.5)
							_,_,_,observed_point,_,_,_ = q.get()
							if type(observed_point) != int:
								break
						observed_point = observed_point[0,0]
						observed_point = observed_point * 1000
						observed_point = np.hstack((observed_point,np.array([1.0])))
						observed_points.append(np.array(observed_point))
					else:
						pass
		which_slide.value = 4
		robot.move_p([500,0,0,pi,0,0],axes = 'sxyz')
		robot.gripper_move(50)
		robot.move_j(robot.q_home)
		# print(measured_points, observed_points[1])
		H = np.linalg.lstsq(observed_points, measured_points,rcond=None)[0]
		# np.save('calibration_H.npy',H)
		print(H.T)
	def s4(q,which_slide): # tic-tac-toe
		robot = self.arm
		board = Board()
		# Please clear the board
		which_slide.value = 6
		board.set_borad([[0, 0, 0], [0, 0, 0], [0, 0, 0]])

	    # who first ?

		# detect ycb cube. If Id = 0: people first; elif Id = 1:AI first
		# X always first


		for _ in range(10):
			_,_,_,_,ids,_,colors = q.get()
		qr = 0
	  	while qr <=10:
	  		_,_,_,_,ids,_,colors = q.get()
	  		if type(ids) != int:
	  			for i in range(ids.shape[0]):
	  				if ids[i,0] == 0 and colors[i] == 'yellow':
	  					qr = qr + 1
		self.robot_say_ok() # put all cubes good

		which_slide.value = 7
		# who first 
		red = 0
		blue = 0
		while True:
			_,_,_,_,ids,_,colors = q.get()
			if type(ids) != int:
				for i in range(ids.shape[0]):
					if colors[i] == 'red':
						red = red + 1
					elif colors[i] == 'blue':
						blue = blue + 1
			if red >= 10:
				ai = tictactoe_ai('O')
				player_chess = 'X'
				Ai_chess = 'O'
				turn = 'player'
				break
			if blue >=10:
				ai = tictactoe_ai('X')
				player_chess = 'O'
				Ai_chess = 'X'
				turn = 'AI'
				break
		# Robot show ok
		self.robot_say_ok()
		old_tvecs = []
		ids_have_played = []
		# [0.10313712, 0.04647613, 0.27434524] action:1
		# [0.01161504, 0.04782838, 0.28138322] action:2
		# [-0.08122865,  0.04817742,  0.28469329] action:3
		# [ 0.10560361, -0.01772023,  0.34650862] action:4
		# [ 0.01232106, -0.01798491,  0.35440032] action:5
		# [-0.08081311, -0.01733993,  0.35411071] action::6
		# [ 0.10612728, -0.08337802,  0.41365185] action:7
		# [ 0.01310125, -0.08178238,  0.41239749] action:8
		# [-0.08000739, -0.08209023,  0.4145219 ] action:9
		ori_tvecs = [[[0],[0.10313712, 0.04647613, 0.27434524]],
			    [[1],[0.01161504, 0.04782838, 0.28138322]],
			    [[2],[-0.08122865,  0.04817742,  0.28469329]],
			    [[3],[ 0.10560361, -0.01772023,  0.34650862]],
			    [[4],[ 0.01232106, -0.01798491,  0.35440032]],
			    [[5],[-0.08081311, -0.01733993,  0.35411071]],
			    [[6],[ 0.10612728, -0.08337802,  0.41365185]],
			    [[7],[ 0.01310125, -0.08178238,  0.41239749]],
			    [[8],[-0.08000739, -0.08209023,  0.4145219 ]]]

		# mian loop
		while True:
			new_tvecs = []
			res = board.get_winner()
			if res == 0:
				# X win
				winner = 'X'
				if player_chess == 'X':
					which_slide.value = 10
				else:
					which_slide.value = 11
				time.sleep(2)
				break
			elif res == 1:
				# O win
				winner = 'O'
				if player_chess == 'O':
					which_slide.value = 10
				else:
					which_slide.value = 11
				time.sleep(2)
				break
			elif res == 2:
				# check ava
				if not board.check_avalible_action():
					winner = 'no'
					break
				else: # go on
					pass

			if turn == 'player':
				#board.print_board()
				which_slide.value = 8
				
				turn = "AI"
				# play ...
				
				for _ in range(10):
					_,_,_,_,ids,_,colors = q.get()
				qr = 0
		  		while qr <=10:
		  			_,_,_,_,ids,_,colors = q.get()
		  			if type(ids) != int:
		  				for i in range(ids.shape[0]):
		  					if ids[i,0] == 0 and colors[i] == 'yellow':
		  						qr = qr + 1
				self.robot_say_ok()		

				which_slide.value = 5
				not_get_new_action = True
				while not_get_new_action:
					_,_,rvec,tvec,ids,_,colors = q.get()
					print(colors)
					for i in range(tvec.shape[0]):
						if colors[i] == 'green' or colors[i] == 'yellow':# or (ids[i,0] == 0 and colors[i] == 'yellow'):
							continue
						new_tvecs.append(tvec[i,0])
					if len(new_tvecs) == len(old_tvecs) + 1:
						not_get_new_action = False
					else:
						new_tvecs = []

				
				# delete
				# len(old_tvecs) = len(new_tvecs) - 1
				# no new action list
				#print(new_tvecs,old_tvecs)
				js = []
				new_actions = copy.copy(new_tvecs)
				for i in range(len(old_tvecs)):
					# delete old tvecs
					for j in range(len(new_tvecs)):
						# get diff dist = numpy.sqrt(numpy.sum(numpy.square(old_tvecs[i] - new_tvecs[j])))
						print(np.sqrt(np.sum(np.square(old_tvecs[i] - new_tvecs[j]))))
						if np.sqrt(np.sum(np.square(old_tvecs[i] - new_tvecs[j]))) < 0.05:
							new_actions[j] = 0
				print(new_actions)
				for i in range(len(new_actions)):
					if type(new_actions[i]) != int:
						new_action = new_actions[i]
						break
				print("new_action:",len(new_action))
				# from tvec -> action
				# ori_tvecs(known)
				for i in range(len(ori_tvecs)):
					print(np.sqrt(np.sum(np.square(np.array(ori_tvecs[i][1]) - new_action))))
			 		if np.sqrt(np.sum(np.square(np.array(ori_tvecs[i][1]) - new_action))) < 0.05: # close
			 			# find action
						action = ori_tvecs[i][0]
				 		break
		 		print(action)
				board._move(int(action[0]),player_chess)
				board.print_board()
				old_tvecs = new_tvecs
				
			elif turn == "AI":
				which_slide.value = 9
				#board.print_board()
				#print("AI's turn...")
				action = ai.think(board)
				loop = True
				which_slide.value = 5
				while loop:
					_,_,rvec,tvec,_,_,colors = q.get()

					if type(tvec) is not int:
						for i in range(tvec.shape[0]):
							if colors[i] == 'green' and (tvec[i,0,0] > 0.11) and (ids[i,0] not in ids_have_played):
								cam_posi1 = np.array(tvec[i,0,:])*1000
								ids_have_played.append(ids[i,0])
								loop = False
								break
				which_slide.value = 9
				print('green ids:',ids_have_played)
				cam_posi1 = np.hstack((cam_posi1,np.array([1.0])))
				rob_posi1 = np.dot(H,cam_posi1.T)
				cam_posi2 = np.array(ori_tvecs[action][1])*1000
				cam_posi2 = np.hstack((cam_posi2,np.array([1.0])))
				rob_posi2 = np.dot(H,cam_posi2.T)
				robot.gripper_move(40)
				robot.move_p([rob_posi1[0]-5,rob_posi1[1],100,pi,0,0])
				robot.move_p([rob_posi1[0]-5,rob_posi1[1],0,pi,0,0])
				robot.gripper_grasp(10,0.1,100)
				robot.move_p([rob_posi1[0]-5,rob_posi1[1],100,pi,0,0])
				
				robot.move_p([rob_posi2[0]-5,rob_posi2[1],100,pi,0,0])
				robot.move_p([rob_posi2[0]-5,rob_posi2[1],0,pi,0,0])
				
				robot.gripper_move(40)
				robot.move_p([rob_posi2[0]-5,rob_posi2[1],100,pi,0,0])
				robot.move_j(robot.q_home)
				board._move(int(action),Ai_chess)
				turn = 'player'
				board.print_board()
	def s5(q,which_slide): # toy grasp
		robot = self.arm
		which_slide.value = 12

		for _ in range(10):
			_,_,_,_,ids,_,colors = q.get()
		qr = 0
		while qr <=10:
			_,_,_,_,ids,_,colors = q.get()
			if type(ids) != int:
				for i in range(ids.shape[0]):
					if ids[i,0] == 0 and colors[i] == 'yellow':
						qr = qr + 1
		self.robot_say_ok()		

		loop = True
		while loop:
			for _ in range(20):
				img,_,_,_,_,_,_ = q.get()
			img1 = copy.copy(img)

			res = G.predict(img)
			u = res[0]
			v = res[1]
			if res[-1] < 0.9:
				return
			
			#cv2.circle(img1,(u,v), 10, (0,255,255), -1)
			#cv2.imshow('i',img1)
			#if cv2.waitKey(1) == ord('q'):
			#	breaks
			
			z = 300 - (v - 360)*150/360
			x = (u-624.79)*((z/1000.0)/931.69)*1000
			y = (v-360.52)*((z/1000.0)/931.46)*1000
			p = np.dot(H,np.array([x,y,z,1]))
			robot.gripper_move(50)
			robot.move_p([p[0]+15,p[1],100,pi,0,res[3]])
			robot.move_p([p[0]+15,p[1],10,pi,0,res[3]])
			robot.gripper_grasp(1,0.1,100)
			#print(robot.gripper_width())
			robot.move_p([p[0]+15,p[1],100,pi,0,res[3]])
			robot.move_p([300,250,300,pi,0,0])
			robot.gripper_move(50)
			#print(res)
	def s6(q,which_slide,show_num): # data cali
		robot = self.arm
		which_slide.value = 13
		loop = True
		buff_size = 30
		q_green_buff = [0]*buff_size
		q_red_buff = [0]*buff_size
		imgs = []
		while loop:
			img,_,rvecs,tvecs,_,_,colors = q.get()
			del q_green_buff[0]
			del q_red_buff[0]
			q_green_buff.append(0)
			q_red_buff.append(0)
			if type(tvecs) != int:
				for i in range(tvecs.shape[0]):
					if colors[i] == 'green':
						q_green_buff[-1] = 1
						break
					elif colors[i] == 'red':
						q_red_buff[-1] = 1
						break
				q_green_buff_np = np.array(q_green_buff)
				q_red_buff_np = np.array(q_red_buff)
				if np.sum(q_green_buff_np) == (buff_size - 3):
					q_green_buff = [0]*buff_size
					# take photo
					imgs.append(img)
					show_num.value = show_num.value + 1
				elif np.sum(q_red_buff_np) == (buff_size - 3):
					q_red_buff = [0]*buff_size
					# end
					imgs = np.array(imgs)
					np.save('Data.npy',imgs)
					loop = False
		show_num.value = 0
	def s7(q,which_slide): # waste sorting
		robot = self.arm
		which_slide.value = 14
		# start
		for _ in range(10):
			_,_,_,_,ids,_,colors = q.get()
		qr = 0
	  	while qr <=10:
	  		_,_,_,_,ids,_,colors = q.get()
	  		if type(ids) != int:
	  			for i in range(ids.shape[0]):
	  				if ids[i,0] == 0 and colors[i] == 'yellow':
	  					qr = qr + 1
		self.robot_say_ok()
		# er
		loop = True
		end_count = 0
		g = 0
		r = 0
		o = 0
		z = 0
		while loop:
			for _ in range(20):
				_,_,rvecs,tvecs,ids,_,colors = q.get()
			if type(tvecs) != int:
				for i in range(tvecs.shape[0]):
					print(tvecs)
					if tvecs[i,0,0] > 0.05:
						end_count = 0
						if colors[i] == 'green':
							rob_posi2 = [600,100]
							z = g*25
							g = g + 1
							go = True
						elif colors[i] == 'red':
							rob_posi2 = [550,100]
							z = r*25
							r = r + 1
							go = True
						elif colors[i] == 'orange':
							rob_posi2 = [500,100]
							z = o*25
							o = o + 1
							go = True
						elif colors[i] == 'yellow' and ids[i,0] == 0:
							loop = False
						else:
							continue
						if go :
							cam_posi1 = np.array(tvecs[i,0,:])*1000
							cam_posi1 = np.hstack((cam_posi1,np.array([1.0])))
							rob_posi1 = np.dot(H,cam_posi1.T)
							robot.gripper_move(40)
							robot.move_p([rob_posi1[0]-10,rob_posi1[1],100,pi,0,0])
							robot.move_p([rob_posi1[0]-10,rob_posi1[1],0,pi,0,0])
							robot.gripper_grasp(10,0.1,100)
							robot.move_p([rob_posi1[0]-10,rob_posi1[1],100,pi,0,0])
							
							robot.move_p([rob_posi2[0],rob_posi2[1],100,pi,0,0])
							robot.move_p([rob_posi2[0],rob_posi2[1],z,pi,0,0])
							
							robot.gripper_move(40)
							robot.move_p([rob_posi2[0],rob_posi2[1],100,pi,0,0])
							robot.move_j(robot.q_home)
							go = False
					else:
						end_count = end_count + 1
			else:
				end_count = end_count + 1
			#print(end_count)
			if end_count >= 10:
				loop = False
	def close():
		robot.__del__()
		os.system("kill -9 $(ps|grep 'python'|awk '{print $1}')")# kill realsense process

	def robot_say_no(self):
		robot = self.arm
		home = copy.copy(robot.q_home)
		home[6] = home[6] - pi/8
		robot.move_j(home)
		home[6] = home[6] + pi/4
		robot.move_j(home)
		home[6] = home[6] - pi/4
		robot.move_j(home)
		robot.move_j(robot.q_home)

	def robot_say_ok(self):
		robot = self.arm
		home = copy.copy(robot.q_home)
		home[5] = home[5] - pi/8
		robot.move_j(home)
		robot.move_j(robot.q_home)
		robot.move_j(home)
		robot.move_j(robot.q_home)
    def task_display(self):
    	img_location = mp.Array('i',[200,200])
		qr_location = mp.Array('f',[0.0,0.0,0.0])
		which_slide = mp.Value('i',0)
		img_show_control = mp.Value('i',0)
		show_num = mp.Value('i',0)
		q = mp.Queue(maxsize=10)
		q_qr = mp.Queue(maxsize=10)

		p1 = mp.Process(target=self.show_slides,args=(q,which_slide,img_show_control,img_location,show_num))
		p2 = mp.Process(target=self.realsense_run,args=(qr_location,q))
		p3 = mp.Process(target=self.show_qr,args=(q,))
		p1.start() # start slides
		p2.start()
		#p3.start()

		self.arm.gripper_home()
		self.arm.go_home()
		
		#s1(which_slide,img_show_control,img_location) # demo

		#s2(q,which_slide) # grasp

		#s3(q,which_slide) # calibration

		#s4(q,which_slide) # TTT

		s5(q,which_slide) # toy grasp

		#s6(q,which_slide,show_num) # data collect

		#s7(q,which_slide) # waste sorting
		close()
