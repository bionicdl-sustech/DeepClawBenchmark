#!/usr/bin/env python2

import rospy
from tf import TransformBroadcaster, TransformerROS, transformations as tfs
import tf, os, time
from geometry_msgs.msg import Transform
import numpy as np

# Uncomment below to verify handeye calibration accuracy in Rviz

# For Franka, Run the following script before publishing the handeye matrix
# os.system("gnome-terminal -e roslaunch panda_moveit_config panda_control_moveit_rviz.launch  load_gripper:=true robot_ip:=192.168.31.159")
# time.sleep(3)
# os.system("gnome-terminal -e roslaunch realsense2_camera rs_rgbd.launch")
# time.sleep(3)

# For UR5
os.system("gnome-terminal -x roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=192.168.1.27 kinematics_config:=$(rospack find ur_calibration)/etc/ur5_calibration.yaml")
time.sleep(3)
os.system("gnome-terminal -x roslaunch realsense2_camera rs_rgbd.launch")
time.sleep(3)
os.system("gnome-terminal -x rosrun rviz rviz")
time.sleep(3)

d = np.load("../../data/calibration_data/ur5-realsense.npz")
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

print("R: %s"%R)
print("t: %s"%t)
H = np.concatenate([np.concatenate([R,t.reshape(3,1)],axis=1),np.array([0, 0, 0, 1]).reshape(1,4)])
np.savetxt("camera_pose.txt",H)

# TODO: calculate the quaternion from rotation matrix
al, be, ga = tf.transformations.euler_from_matrix(R, 'sxyz')
q = tf.transformations.quaternion_from_euler(al, be, ga, axes='sxyz')

# broadcast the hand eye transformation to topic /tf
rospy.init_node('handeye_calibration_publisher')
print("Publishing handeye matrix!")
broad = TransformBroadcaster()
rate = rospy.Rate(50)
while not rospy.is_shutdown():
    broad.sendTransform((t[0],t[1],t[2]), (q[0],q[1],q[2],q[3]), rospy.Time.now(), "camera_color_optical_frame", "base_link")  # takes ..., child, parent
    rate.sleep()
