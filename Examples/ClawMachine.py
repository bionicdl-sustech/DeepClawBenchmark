import os
import sys

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(root_path)
# print(root_path)

from Functions.ClawMachine.predictor import Predictor
from Functions.ClawMachine.Functions import *
from Functions.ClawMachine.success_label import *
from ToolKit.DataCollector import ImagePublisher, TimePublisher, Monitor

image_publisher = ImagePublisher('image_pub')
time_publisher = TimePublisher('time_pub')
data_collector = Monitor('ClawMachineTask1')
image_publisher.registerObserver(data_collector)
time_publisher.registerObserver(data_collector)

def subtask_display(perception_system, manipulation_system, args, is_debug=False):
    # args initial
    camera = perception_system[0]
    robot_arm,  robot_gripper = manipulation_system[0], manipulation_system[1]
    Network, NUM_BOXES, WIDTH, crop_box, WorkSpace, ite = args[0], args[1], args[2], args[3], args[4]

    robot_arm.goHome()
    color_image, _ = camera.getImage()
    croped_image = color_image[crop_box[0]:crop_box[2], crop_box[1]:crop_box[3]]

    if is_debug:
        image_publisher.setData(color_image)
        image_publisher.setData(croped_image)

    robot_gripper.openGripper()

    # end to end network
    start_time = time.time()
    candidates_theta, candidates_probability, patches, boxes = end2end_display(image_publisher, camera, [Network, croped_image, NUM_BOXES, WIDTH], is_debug)
    end_time = time.time()
    if is_debug:
        time_publisher.setData([str(ite)+'end2end', end_time - start_time])

    # multiple points motion planning step
    start_time = time.time()
    pick_location, place_location, best_theta = multiple_points_motion_planning(image_publisher, [robot_arm, robot_gripper],
                                                                                [candidates_theta, candidates_probability,
                                                                                 croped_image, boxes,
                                                                                 NUM_BOXES, WIDTH, crop_box])
    end_time = time.time()
    if is_debug:
        time_publisher.setData([str(ite)+'multiple_points_motion_planning', end_time - start_time])

    # execution step
    start_time = time.time()
    execution_display(image_publisher, [robot_arm, robot_gripper], [pick_location, place_location])
    end_time = time.time()
    if is_debug:
        time_publisher.setData([str(ite)+'execution', end_time - start_time])

    color_image2 = camera.getImage()
    image_publisher.setData(color_image2)
    grasp_label = success_label(color_image, color_image2)

    return grasp_label

def task_display(perception_system, manipulation_system, is_debug=False):
    # parameters initial
    NUM_BOXES = 20
    WIDTH = 120
    Network = Predictor(root_path+'/Functions/ClawMachine/checkpoint')
    crop_box = [320, 140, 1000, 720]
    WorkSpace = [-0.21, -0.37, 0.21, -0.68]

    # robot system
    robot_arm = manipulation_system['Arm']
    robot_gripper = manipulation_system['End-effector']

    # sensors
    camera = perception_system['Camera']

    Objectcount = 0
    for i in range(20):
        grasp_label = subtask_display([camera], [robot_arm, robot_gripper],
                                      [Network, NUM_BOXES, WIDTH, crop_box, WorkSpace, i], is_debug)
        if grasp_label==1:
            Objectcount+=1
        # raw_input('waiting...')
    print(Objectcount)
