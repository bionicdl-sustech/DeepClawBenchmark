'''
Kinematic picking
'''
from os.path import dirname, abspath
sim_path = dirname(dirname(dirname(abspath(__file__))))
scene_path = sim_path + '/scene/'
import sys
sys.path.append(sim_path)
from src.camera import Camera
from src.env import Env
from src.franka import Franka
from pyrep.objects.shape import Shape
import numpy as np
import cv2

def scene(scene_file_name):
    '''
    return abs dir of scene file 
    '''
    return scene_path + scene_file_name
if __name__ == '__main__':
    # open v-rep and launch BaseScene.ttt file
    env = Env(scene('Kine_picking.ttt'))
    # start simulation
    env.start()

    # franka
    franka = Franka()
    # set franka to home joints 
    franka.home(env)

    # TODO: generate a series of waypoints form the letters in "KINEMATICS"
    x,y,z = franka.get_position()
    '''
    rotate along Y axis 45 degrees
    and move to robot frame
    '''
    rotation = np.array([
        [np.cos(np.pi/4),0,np.sin(np.pi/4),x],
        [0,1,0,y],
        [-np.sin(np.pi/4),0,np.cos(np.pi/4),z],
        [0,0,0,1]
    ])
    '''
    letter_range: the letters should be within the bounding box below

      0.2
    -------
    |     |
    |  *  | 0.3 ----> y
    |     |
    -------
       |
       |
       | x

    z = 0.7
    
    letter_range=[
        [-0.15,0.1,0.7,1],
        [-0.15,-0.1,0.7,1],
        [0.15,-0.1,0.7,1],
        [0.15,0.1,0.7,1],
    ]
    '''
    # TODO: here we give an example of letter "I", your task is to form a letter from "KINEMATICS"
    letter_I_targets = [
        [-0.15,-0.1,0.7,1],
        [-0.15,0.1,0.7,1],
        [-0.15,0,0.7,1],
        [0.15,0,0.7,1],
        [0.15,-0.1,0.7,1],
        [0.15,0.1,0.7,1]
    ]

    for i,p in enumerate(letter_I_targets):
        if(i==0):
            franka.clear_path = True
        else:
            franka.clear_path = False
        rp = rotation@np.array(p)
        
        franka.move(env,rp[:3],euler=[0,np.radians(180),0])
    franka.home(env)
    input('go')
    env.stop()
    env.shutdown()
