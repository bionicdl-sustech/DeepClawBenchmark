import socket
import time
import numpy as np

robot_ip = "192.168.1.27" # The remote robot_ip
port = 30003
print("Starting Program of drawing 4 circles using servoj!")

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(10)
s.connect((robot_ip, port))

##################################################################
# TODO: Testing servoj and speedl
# set time step of servoj
# set the scale of wait time for execution of each waypoint, the small this number is, the fast the move is
# the wait time should be less the dt, otherwise the movement will stop at every waypoints
dt = 0.01
scale = 0.7
for j in range(1):
    # set radius of the jth circle
    R = 0.08 - 0.01*j
    t1 = time.time()
    for i in range(1100):
        if i>1000:
            theta = 3.1415/500 * 1000
        else:
            theta = 3.1415/500 * i
        x = -0.0253 + R*np.cos(theta)
        y = -0.53275418 + R*np.sin(theta)
        tcp_command = "servoj(get_inverse_kin(p [%s, %s,  0.50, 3.14030481,  0.06973561, 0.0102991]), t=%s,gain=100)\n"%(x,y, dt)
        l = s.send(str.encode(tcp_command))
        time.sleep(dt*scale)
    t2=time.time()
    print("Time cost of drawing %sth circle: %s"%(j, t2-t1))

# speedl([X, Y, Z Rx, Ry, Rz], accl, time)
dt = 0.1
scale = 0.7
for i in range(8):
    tcp_command = "speedl([-0.05, 0, 0, 0, 0, 0],1.0,%s)\n"%(dt)
    l = s.send(str.encode(tcp_command))
    time.sleep(dt*scale)

s.close()
##################################################################
# TODO: Position based visual servoing
# Task: Tool0 of UR5 follows a chessboard
import numpy as np
import time
import cv2
import cv2.aruco as aruco
import urx
from driver.arms.UR5Controller import UR5Controller
robot = UR5Controller("/config/arms/ur5.yaml")
robot.load_calibration_matrix()
from driver.sensors.camera.RealsenseController import RealsenseController
camera = RealsenseController()
from modules.calibration.Calibration3D import *
from scipy.spatial.transform import Rotation as R
from numpy.linalg import inv

rob = urx.Robot("192.168.1.27")

# get intrinsics of Realsense
intrinsics = camera.get_intrinsics()
fx = intrinsics[0]
fy = intrinsics[1]
ppx = intrinsics[2]
ppy = intrinsics[3]
mtx = np.array([[fx, 0, ppx],
                [0,  fy, ppy],
                [0,  0,  1]])
dist = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

# set up for chessboard detection
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((8*11,3), np.float32)
objp[:,:2] = np.mgrid[0:11,0:8].T.reshape(-1,2)*0.02

# setup handeye matrix and the desired transformation between end effector and object (chessboard)
bMc = np.load("data/calibration_data/handeye_matrix_UR5Realsense.npy")
edMo = np.array([[1, 0, 0, -0.4],[0, 1, 0, 0.0], [0, 0, 1, 0.5], [0,0,0,1]])
oMed = inv(edMo)

dt = 0.3
scale = 0.6
for i in range(1000):
    t0 = time.time()
    frame = camera.get_frame()
    img = frame.color_image[0]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (11,8),None)
    if ret == True:
        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        ret,rvec, tvec = cv2.solvePnP(objp, corners2, mtx, dist) # cMo
    t1 = time.time()
    r = R.from_euler('xyz',[rvec[0][0],rvec[1][0],rvec[2][0]],degrees=False).as_dcm()
    # calculate the pose of chessboard w.r.t. camera
    cMo = np.concatenate([np.concatenate([r,tvec],axis=1),np.array([0, 0, 0, 1]).reshape(1,4)])
    # calculate the desired pose of end effector w.r.t. base
    bMed = np.matmul(np.matmul(bMc, cMo), oMed)
    t_desired = R.from_dcm(bMed[:3,:3]).as_rotvec()
    p_desired = np.array([bMed[0,3], bMed[1,3],bMed[2,3],t_desired[0],t_desired[1],t_desired[2]])
    p_current = rob.get_pose().get_pose_vector()
    t2 = time.time()
    print("Computing time cost: %s,%s, %s"%(t1-t0, t2-t1))
    dp = (p_desired - p_current)/1
    dp[3:] = dp[3:]- (dp[3:]>3.1415926)*2*3.1415926
    dp[3:] = dp[3:]+ (dp[3:]<-3.1415926)*2*3.1415926
    # print(dp[3:])
    p = p_current + 0.5*dp
    # tcp_command = "speedl([%s, %s, 0, %s, %s, %s],1.0,%s)\n"%(dp[0],dp[1],dp[3],dp[4], dp[5],dt)
    tcp_command = "servoj(get_inverse_kin(p [%s, %s, 0.4, %s, %s, %s0]), t=%s,gain=100)\n"%(p[0],p[1],p[3],p[4], p[5], dt)
    l = s.send(str.encode(tcp_command))
    # time.sleep(dt*scale)

cv2.imshow("hi",color_image)
cv2.waitKey()
cv2.destroyAllWindows()

def chessboard_detector():
    def draw(img, corners, imgpts):
        corner = tuple(corners[0].ravel())
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (0,0,0), 3)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 3)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 3)
        return img
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((8*11,3), np.float32)
    objp[:,:2] = np.mgrid[0:11,0:8].T.reshape(-1,2)*0.02
    axis = np.float32([[4,0,0], [0,4,0], [0,0,4]]).reshape(-1,3)*0.02
    for i in range(1):
        frame = camera.get_frame()
        img = frame.color_image[0]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (11,8),None)
        if ret == True:
            print("Found")
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            # Find the rotation and translation vectors.
            ret,rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
            img = cv2.drawChessboardCorners(img, (11,8), corners2,ret)
            img = draw(img,corners2,imgpts)
            cv2.imshow('img',img)
            k = cv2.waitKey(0) & 0xFF
        else:
            print("Fail")
    cv2.destroyAllWindows()
