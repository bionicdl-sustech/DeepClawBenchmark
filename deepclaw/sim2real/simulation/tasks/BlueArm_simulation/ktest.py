from os.path import dirname, abspath
from os import system, environ
sim_path = dirname(dirname(dirname(dirname(abspath(__file__)))))
scene_path = sim_path + '/simulation/scene/'
import sys
sys.path.append(sim_path)
from simulation.src.camera import Camera
from simulation.src.env import Env
from pyrep.robots.arms.arm import Arm
from pyrep.robots.end_effectors.gripper import Gripper
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.objects.shape import Shape
import numpy as np
import cv2
import copy

def scene(scene_file_name):
    # return abs dir of scene file 
    return scene_path + scene_file_name

class BlueArm(Arm):
    def __init__(self, Armname):
        super().__init__(0, Armname, 5)


class BlueArmGripper(Gripper):
    def __init__(self, Armname):
        super().__init__(0, Armname,
                         [Armname+'_gripper_joint'])

class Kinect(VisionSensor):
    def __init__(self):
        super().__init__('kinect')
        # enable camera sensor
        #self.set_explicit_handling(1)
        #self.handle_explicitly()
        # compute vision sensor intrinsic matrix
        # [ax 0  u0
        #  0  ay v0
        #  0  0  1]
        self.ax = 2*np.tan(np.radians(self.get_perspective_angle()/2))/self.get_resolution()[0]
        self.ay = 2*np.tan(np.radians(self.get_perspective_angle()*self.get_resolution()[1]/self.get_resolution()[0]*2))/self.get_resolution()[1]
        self.u0 = self.get_resolution()[0]/2 # u0
        self.v0 = self.get_resolution()[1]/2 # v0
        self.H = np.array([[0,1,0,1.1],
                           [1,0,0,0],
                           [0,0,-1,1.8],
                           [0,0,0,1]])
    def capture_bgr(self):
        img = cv2.cvtColor(self.capture_rgb(),cv2.COLOR_RGB2BGR)*255
        return np.array(img,dtype=np.uint8)

    def uv2XYZ(self,depth_img,u,v):
        Z = depth_img[v,u]
        return np.array([Z*(u-self.u0)*self.ax, Z*(v-self.v0)*self.ay, Z, 1])

if __name__ == "__main__":
    env = Env(scene('BlueArm.ttt'))
    env.start()

    left_arm = BlueArm('LeftBlueArm')
    right_arm = BlueArm('RightBlueArm')
    left_arm_gripper = BlueArmGripper('LeftBlueArm')
    right_arm_gripper = BlueArmGripper('RightBlueArm')

    env.stop()
    env.shutdown()