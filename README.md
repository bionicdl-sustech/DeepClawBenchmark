# DeepClaw <!-- omit in toc -->

paper | poster | video

![](https://github.com/bionicdl-sustech/DeepClawBenchmark/blob/master/docs/figs/fig-PaperOverview.png)

We present DeepClaw as a reconfigurable benchmark of robotic hardware and task hierarchy for robot learning. The DeepClaw benchmark aims at a mechatronics perspective of the robot learning problem, which features a minimum design of robot cell that can be easily reconfigured to host robot hardware from various vendors, including manipulators, grippers, cameras, desks, and objects, aiming at a streamlined collection of physical manipulation data and evaluation of the learned skills for hardware benchmarking. We provide a detailed design of the robot cell with readily available parts to build the experiment environment that can host a wide range of robotic hardware commonly adopted for robot learning. We also propose a hierarchical pipeline of software integration, including localization, recognition, grasp planning, and motion planning, to streamline learning-based robot control, data collection, and experiment validation towards shareability and reproducibility.

DeepClaw is a benchmarking system for robot manipulation designed to be modularized, extendable, and easy to use on real robots and the environment. As shown in Fig.1, DeepClaw consists of four components:

1. A standardized robot cell design.
2. A set of unified driver interfaces that serves as a connection layer between the benchmarking algorithms and the hardware, including robot arms, robot hands, visual sensors, tactile sensors.
3. A collection of baseline algorithms for segmentation, recognition, grasp planning, and motion planning.
4. A pipeline for task definition and functional integration.

For a detailed decription of DeepClaw and benchmarking tasks, please visit the [website of DeepClaw](https://bionicdl-sustech.github.io/DeepClawBenchmark/)

# Quick Start <!-- omit in toc -->

To be continued ... (the kind of code that anyone needs to get started with DeepClaw)
