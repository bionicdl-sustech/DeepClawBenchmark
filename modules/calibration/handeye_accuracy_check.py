import numpy as np
d = np.load("data/calibration_data/franka-realsense.npz")
observed_pts = d['arr_0']
measured_pts = d['arr_1']

# TODO: calculate the transformation between two sets of point A and B
def get_rigid_transform(A, B):
    assert len(A) == len(B)
    N = A.shape[0]; # Total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - np.tile(centroid_A, (N, 1)) # Centre the points
    BB = B - np.tile(centroid_B, (N, 1))
    H = np.dot(np.transpose(AA), BB) # Dot is matrix multiplication for array
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0: # Special reflection case
       Vt[2,:] *= -1
       R = np.dot(Vt.T, U.T)
    t = np.dot(-R, centroid_A.T) + centroid_B.T
    return R, t

R, t = get_rigid_transform(observed_pts, measured_pts)
########################################################
# verify handeye calibration accuracy with acruco marker on the table
import numpy as np
import time
import cv2
import cv2.aruco as aruco
from driver.sensors.camera.RealsenseController import RealsenseController
import matplotlib.pyplot as plt

camera = RealsenseController()

frame = camera.get_frame()
color_image = frame.color_image[0]
depth_image = frame.depth_image[0]
img_display = color_image.copy()
gray_img = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
parameters =  aruco.DetectorParameters_create()
corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_img,aruco_dict,parameters=parameters)

u = int((corners[0][0][0,0]+corners[0][0][2,0])/2)
v = int((corners[0][0][0,1]+corners[0][0][2,1])/2)
x=np.multiply(u-640.5963134765625,0.96/916.4114990234375)
y=np.multiply(v-368.7446594238281,0.96/914.789306640625)
p = np.matmul(R, np.array([x,y,0.96]).reshape([3,1])) + t.reshape([3,1])
# plt.imshow(color_image)
# plt.show()

# move the arm to the detected acruvo marker
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
               anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "panda_arm"
group = moveit_commander.MoveGroupCommander(group_name)

pose_c = group.get_current_pose()
pose_goal = pose_c.pose
pose_goal.position.x = p[0][0]
pose_goal.position.y = p[1][0]
pose_goal.position.z = 0.13
group.set_pose_target(pose_goal)
plan = group.plan()

group.execute(plan)
