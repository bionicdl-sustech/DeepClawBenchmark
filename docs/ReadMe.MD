# DeepClaw

- DeepClaw is a model zoo for robot manipulation
- What is the design theory of robotic integration with a balanced hardware and software choice for engineering applications?
- Cloud-enabled Lean Robotics for Engineering Manipulation
  - Lean Robotics => minimize waste reduction => maximize time utility => in robotic scenario
  - Cloud Architecture => system deployment
  - Model Benchmark => too many choices?
  - Hardware + Software = ? => which one to go?
- What are the deliverables?
  - DeepClaw: a cloud-enabled model zoo for engineering manipulation
  - A complex benchmark of robotic hardware and software
  - A comprehensive benchmark of engineering manipulation
  - A performance evaluation program that has it all
  - Example: 
   - in scenario A, with hardware B and algorithm C, we can expect a X% time reduction
   - compare to previous steps and time cost (Lean Roboic)

## What is DeepClaw
DeepClaw is a benchmarking system for robot manipulation designed to be modularized, extendable, and easy to use
on real robots and the environment. As shown in Fig. 1, DeepClaw consists of four components:
 - A standardized robot cell design.
 - A set of unified driver interfaces that serves as a connection layer between the benchmarking algorithms and the hardware, including robot arms, robot hands, visual sensors, tactile sensors.
 - A collection of baseline algorithms for segmentation, recognition, grasp planning, and motion planning.
 - A pipeline for task definition and functional integration.

  <p align="center"><img src="./fig-PaperOverview.png" width="80%"/></p>
  <p align="center">Figure 1. The design of DeepClaw platform towards a reproducible and shareable benchmark for learning robotic manipulation, including (a) a standardized robot station design, (b) a modular pipeline for functional integration, and (c) exemplified task variations.</p>

## what are the features of DeepClaw
- **[Cloud robotics](./cloud_robotics/ReadMe.md)**: offload local computation, save space and weight of the robot cell. 运算和执行分离,易于升级
- **model zoo**: abundant algorithms.
- **standard workflow for robotic manipulation**: reduces the barriers to usage;
- **reusable components**: reusable algorithms for different tasks.
- **extensible**: new algorithm is easy to add in DeepClaw.
- **open source**:



## what are the benefits of DeepClaw
- shareable and reproducible
    + algorithms comparison platform
    + hardware comparison platform
- reduces the barriers to usage of robot manipulation
    + hardware: the local robots just need physical executive agencies and basic computation, as heavy computation is implementing on cloud
    + hardware: drivers for robots and sensors
    + software: a guideline of robotics manipulation
    + software: complex task can be implemented with existed algorithms.
- abundant tasks of robotics manipulation: a lot of tasks are implemented with DeepClaw framework.

## Why we need DeepClaw
we want to konw how algorithms and hardware performance influence the performance of a task. To compare performance, we need:
- a platform to implement different manipulation tasks
- a platform to implement same task with different methods followed same workflow
- a platform to implement same task with different hardware (cloud robotics)
- a easily extensible platform for new algorithms and hardware


______________________


![](docs/asset/fig-DeepClawOverview.png)

The DeepClaw is a benchmarking model zoo that functions as a Reconfigurable Robotic Manipulation System for Robot Learning. Modern systems for robotic manipulation has gradually evolved to a common configuration of a cell station where multiple hardware and software are usually integrated to perform programmable tasks, such as object manipulation, human-robot interaction, and industrial collaboration. The challenge behind most manipulation-based robotic research and application is usually hidden by the growing complexity when integrating related hardware, such as manipulators, grippers, sensors, and environment with objects for manipulation, as well as software, such as algorithms, APIs, drivers, etc., towards a functional robot system. It is widely recognized among industrial robotics that the total cost of integration is usually at the same level as the price tage of the hardware itself, and may become even higher at many scenarios, where the market of robotic integration is actually slightly higher than the selling of robotic hardware itself. 

For academic research, there are several alternative solutions before DeepClaw.
- **Commercial service providers for robotic integration**: This is a common solution that may require the least of effort at the beginning. But in the end, may not be suitable for research purpose at some point where further developement may not be possible or just too expensive. 
- **Robot Operating System (ROS)**: The development of ROS contributed significantly to the growing adoption of robotic technologies in research and applications, which provides a standardized interface for communication among various robotic devices and software. However, the scope of the ROS is so broad with a relatively steep learning curve for most users, learners, and practioners to get started. 
- **Do-it-yourself (DIY)**: This is also a common solution which usually starts with the dedicated software provided by the hardware supplier, or if the robotic device is developed by the research team from the ground up. With the growing adoption of collaborative robotic devices, the barrier of usage becomes much lower than before. 

With a growing interest in robot learning, where learning-based methods are applied to robotic system to solve advanced manipulation tasks towards human-robot interaction. Several questions becomes imminent for researchers and practitioners:
- Is there a "common" setup of robotic manipulation station? 
- By what "standard" should I pick the specific hardware to build my robot learning station?
- Is there a "pipeline" of intergration so that I can get the system up-and-running and perform real-world experiment?

While there is no "correct" answers to questions such as above, these challenges are commonly faced by many researchers already in this field, or have the interest to experiement with robot learning, which give birth to the development of DeepClaw. Based on the existing collection of robotic hardware at the Bionic Design and Learning Lab at the Southern University of Science and Technology, we aim to develop a sharable and reproducible framework of Robot Manipulation System based on reviews of common setups of robot learning stations in recent research, aiming at building up a pool of hardware benchmarking system that facilitates a comparable metric for advanced robot learning algorithms, which is not yet explored in the current research (and also challenging). 

DeepClaw is hardware-centric model zoo towards a common design to benchmark robot stations through a Robotic Manipulation System, which involves a mechanical design of the DeepClaw Station, a growing collection of drivers that supports various robotic hardware, a cloud architecture that supports training and inference of robot learning experiments, and a collection of game objects for task representation.

# Version History
- v2.0: updated with server-client support.
- v1.5: early design used in XXX.
- v1.0: early design used in XXX.

# Mechanical Design of the DeepClaw Station

After reviewing the robot station designs presented in several recent publications on robot manipulation learning, we proposed the following design using a standardized 9090 extrusion from global supplier such as Misumi and a few parts that can be easily machined by local machine shops to build up the system. The design can be accessed interactively through here. Feel free to contact us if you experience trouble sourcing the design, or have any suggestions to improve.
- AluExtru: 9090 series aluminium extrusion profiles from global supperlier such as Misumi.
  - Misumi Part#: HFS8-9090-630-TPW, HFS8-9090-540-TPW, HFS8-9090-450-TPW.
- AluPlate: simple design peg-hole style aluminium plate for reliable connection, can be easily machined by you local shop.
  - S2x4, S2x5.
- FlangeX: simple flange designs that supports heavy load connection, can be easily machined by your local shop.
  - FlangeRobot: Supports Franka, UR, and AUBO in one design, as of now.
  - FlangeFoot: Support the adjustable wheel.
  - FlangeTube: Support RealSense D400 series, as of now. 
- Others: minimum accessories towards flexibility, robustness, and safety.
  - Tube: where cameras are mounted, can be easily modified based on your camera needs.
  - Handle (Part#: GHHD28-A): for ease of handling and safe usage.
  - Adjustable Wheel (GD-80F): for sturdiness and mobility.
  - TableTop (630x540): aluminium with CNC machined, threaded peg-holes, modify as you need.

![](docs/asset/fig-DeepClaw-MechDesign.PNG)

# Supported Robotic Hardware

Here lists the robotic hardware that are currently supported by the DeepClaw station, all tested with a reasonable coverage of common adoption with a reasonable price range and functional support in most open-source repositories.

![](docs/asset/fig-DeepClaw-SupportedHardware.PNG)

# Pipeline of Integration

For further details, please refer to our recent publication at AIM 2020.

![](docs/asset/fig-DeepClaw-Pipeline.PNG)

# Architecture of Integration



# Game-based Objects for Manipulation

# Citation

arXiv
