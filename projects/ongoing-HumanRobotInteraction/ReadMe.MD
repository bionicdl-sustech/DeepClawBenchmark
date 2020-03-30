# Learning Human Robot Interaction

Project Admin: Wang Haokun

# Research Motivation

(based on facts that you collected from all sources possible)

- Human robot interaction (HRI) remains a key problem towards intuitive intelligent agent for physical interaction.
- Soft interaction between the fingers from the robot and human with the objects is a critical problem towards cognitive HRI system.
- How can soft robotics system be used for HRI
  - Soft finger (Omni-adaptive: passive)
  - Soft control (Franka force-control robot: active)
  - Unstructured object for manipulation (non-rigid => unstructured)
- Information Exchange
  - Information Balance (in Humans view and Robot view): both use soft-finger gripper, object and end-effector position info,
  - Information Sharing (direction of data flow): only share humans' information, only share robot’s information, share information with each other.
  - Remote vs. Proximate: data are separated spatially or temporally.

# Research Question

- **What/How/Why** is **something that you are very interested** for **some target** happen or interact in **some scenarion**?
  - What's your version of Research Question?
- How can we help robot to interact with physical objects in a way like human?
  - Soft touch
    - safety for human, safety for robot
    - object adaptation
    - sensing the dynamics of physical interaction (touch)
  - Learning?
  - Human-Robot Interaction
  - ...

Research Issues
- Is it possible to solve more complex task for both humans and robots by sharing information each other?
- How to transfer the perceptual information between humans and robots in a more efficient way?
- What type of information plays the most important role when executing certain task?

# Related Work

- Why human are better than robots?
  - gradually converge to teleoperation and visuomotor learning
  - Why human is better? i.e in motion planning, object manipulation, why?
  - Why robots are bad at tasks like this?
- Why vision is so important?
  - gradually converge to object detection, recognition algorithms
- Why touching is so important?
  - gradually converge to unstructured exploration
- Why soft is so important?
  - gradually converge to the importance of involving softness in HRI

----

- Teleoperation = Vision Servoing Control (real-time feedback control)
  - Vision servoing control is a kind of teleoperation implementations.
  - Teleoperation presents in most remote interaction situations.
  - Vision is the best way to implement real-time control under our conditions.
- Soft-finger with Tactile Sensors
  - For information balance (because hands of human are tactile)
  - Review of some classical and recent popular tactile sensor researches.
  - Review of some soft-finger design.
- Objects Detection Algorithms
  - Vision information produced by objects detection algorithms (using deep learning or just traditional methods).
  - Objects set review (YCB, toys, trash).

# Theoretical Modeling

- Robot arm
  - I: motor command
  - O: $(w, v)
- robot hand:
  - Open/Close $d_h$: {0,1} or [0,1] 
  - Hand: $(w, v)
  - Some others
- soft fingers
  - Deformation: complex $d_f$
  - Anyway to make it simpler?: 
    - a ref point in the middle of the finger? 
    - Grasping force/dynamics: $d_{grasp} = d_h - d_f$
    - Assumption: known object size
- YCB obejcts
- RGB-D camera ${Image}_i$

human arm
human hand
soft fingers
YCB objects
human eyes

# Experiment Details

## Setup

## Procedure

## Results

# Discussion

# Final Remarks