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
