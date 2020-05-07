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
    env.set_simulation_timestep(0.1)
    env.step_ui()
    env.start()

    # franka
    franka = Franka()
    # set franka to home joints 
    franka.home(env)
    
    # cam 
    cam = Camera()

    # target_plane, cylinder, cubic 
    target_plane = Shape('plane')
    cubic = Shape('obj0')
    cylinder = Shape('obj1')

    for obj in [cubic, cylinder]:
        # franka move above the object
        obj_position = obj.get_position()
        obj_position[2] += 0.1
        input('Press enter to move above the object')
        franka.move(env,obj_position,euler=[0,np.radians(180),0])

        # approch the object
        obj_position[2] -= 0.1
        input('Press enter to approch the object')
        franka.move(env,obj_position,euler=[0,np.radians(180),0])
        
        # grasp the object
        input('Press enter to grasp the object')
        franka.grasp(env,obj)
        
        # lift the object
        obj_position[2] += 0.1
        input('Press enter to lift the object')
        franka.move(env,obj_position,euler=[0,np.radians(180),0])
        
        # transport the object to target position
        target_plane_position = target_plane.get_position()
        target_plane_position[2] += 0.1
        input('Press enter to transport the object to target position')
        franka.move(env,target_plane_position,euler=[0,np.radians(180),0])

        # release the object
        input('Press enter to release the object')
        franka.release(env)
        franka.home(env)

    env.stop()
    env.shutdown()