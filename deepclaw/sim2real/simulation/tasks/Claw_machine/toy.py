from os.path import dirname, abspath
from os import system, environ
sim_path = dirname(dirname(dirname(dirname(abspath(__file__)))))
scene_path = sim_path + '/Simulation/scene/'
import sys
sys.path.append(sim_path)
from Simulation.src.camera import Camera
from Simulation.src.env import Env
from Simulation.src.franka import Franka
from pyrep.objects.shape import Shape
import numpy as np
import cv2
import copy
environ['TF_CPP_MIN_LOG_LEVEL'] = "3" # stop print warning!!!
from DeepClaw.modules.end2end.graspNet.fc_predictor import FCPredictor

def scene(scene_file_name):
    # return abs dir of scene file 
    return scene_path + scene_file_name

def franka_move(start, target, grasp_pose):
    franka.clear_path = True
    start[2] += 0.1
    franka.move(env,start,euler=[0,np.radians(180),grasp_pose])
    start[2] -= 0.07
    franka.move(env,start,euler=[0,np.radians(180),grasp_pose])
    
    for toy in toys:
        if franka.gripper._proximity_sensor.is_detected(toy):
            franka.grasp(env,toy)
            break
        if toy is toys[-1]:
            raise RuntimeError('can not sense toy')
    start[2] += 0.07
    franka.move(env,start,euler=[0,np.radians(180),grasp_pose])
    franka.home(env)
    a = copy.copy(franka.home_joints)
    a[0] += np.pi/2
    franka.move_j(a,env)
    franka.release(env)
    franka.home(env)
    
NUM_THETAS = 9
predictor = FCPredictor(NUM_THETAS*2, './net9/Network9-1000-100')

if __name__ == '__main__':
    env = Env(scene('Claw_machine.ttt'))
    env.start()

    # franka
    franka = Franka()
    
    # cam 
    cam = Camera()

    # toys
    Bird = Shape('Bird')
    Hipp = Shape('Hipp')
    Elephant = Shape('Elephant')
    Penguin = Shape('Penguin')
    box_dest = Shape('box_dest')
    target = Shape('Sphere')
    toys = [Bird, Hipp, Elephant, Penguin]
    dest_position = box_dest.get_position()

    # random exchange the position of toys
    toy_positions = [toy.get_position() for toy in toys]
    arr = np.arange(len(toys))
    np.random.shuffle(arr)
    [toys[i].set_position(toy_positions[arr[i]]) for i in range(len(toys))]

    # set franka to home joints
    franka.home(env)
    
    end = False
    while not end:
        img = cam.capture_bgr()
        ros = img[41:299,114:372] # (258, 258)
        depth_image = cam.capture_depth(in_meters=True)
        ros = cv2.resize(ros, (1280, 720), interpolation=cv2.INTER_CUBIC)
        y_, p_best, grasp_pose = predictor.run(ros)
        x,y,angle = grasp_pose
        possi = p_best.max()
        print('possibility:',possi)
        if possi < 0.8:
            break
        cx,cy = int(x*258/1280+114),int(y*258/720+41) # u:cy, v:cx
        # print(x,y,cx,cy)
        real_position = (cam.H@cam.uv2XYZ(depth_image,cx,cy))[0:3]
        real_position[2] = 1.123
        target.set_position(real_position)
        cv2.circle(img,(cx,cy),5,(0,0,255),5)
        cv2.circle(ros,(x,y),5,(0,0,255),5)
        franka_move(real_position, dest_position, grasp_pose[2]) 

    env.stop()
    env.shutdown()
    
