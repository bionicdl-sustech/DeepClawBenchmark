'''
Test BaseScene.ttt and models in src
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
from pyrep.const import PrimitiveShape
import numpy as np
import cv2

def scene(scene_file_name):
    '''
    return abs dir of scene file 
    '''
    return scene_path + scene_file_name

if __name__ == '__main__':
    # open v-rep and launch BaseScene.ttt file
    env = Env(scene('BaseScene.ttt'))
    # start simulation
    env.start()

    # franka
    franka = Franka()
    # set franka to home joints 
    path = franka.home(env)
    
    # cam 
    cam = Camera()
    # find the path to target
    target = Shape.create(type=PrimitiveShape.SPHERE,
                      size=[0.05, 0.05, 0.05],
                      color=[1.0, 0.1, 0.1],
                      static=True, respondable=False)
    
    for i in range(100):

        # Get a random position within a cuboid and set the target position
        pos = list(np.random.uniform(franka.position_min, franka.position_max))
        target.set_position(pos)
        franka.move(env,position=pos, euler=[0, np.radians(180), 0])
        img = cam.capture_bgr()
        print('Reached target %d!' % i)
    env.stop()
    env.shutdown()
    cv2.imshow('img',img)
    cv2.waitKey(0)

