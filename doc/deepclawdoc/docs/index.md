![DeepClaw-Logo](asset/fig-DeepClaw.png)

# The DeepClaw Benchmark

The DeepClaw is a benchmarking model zoo that functions as a Reconfigurable Robotic Manipulation System for Robot Learning. The main homepage for Julia can be found at [deepclaw.ancorasir.com](https://deepclaw.ancorasir.com/). This is the GitHub repository of DeepClaw source code, including instructions for installing and using DeepClaw, below.

## Resources

- Homepage: https://deepclaw.ancorasir.com/
- Documentation: https://bionicdl-sustech.github.io/DeepClawBenchmark/_build/html/index.html
- Paper explaining DeepClaw: arXiv ([BibTex](#bibliography))
- Papers using DeepClaw: 
  - [arXiv:2003.01584 [cs.RO]](https://arxiv.org/abs/2003.01584)
  - [arXiv:2003.01583 [cs.RO]](https://arxiv.org/abs/2003.01583)
  - [arXiv:2003.01582 [cs.RO]](https://arxiv.org/abs/2003.01582)

## Installation from Source

As of now, DeepClaw framework has been tested with Python 2.7 and Ubuntu 16.04 LTS, with a near-future plan to update to Python 3.x with Ubunti 18.04 LTS.

### Virtual Environment

We recommend using a virtual environment (such as virtualenv) to manage DeepClaw.

Install virtualenv.

    $ pip install -U virtualenv

Create a new virtual environment.

    $ virtualenv -p /usr/bin/python2.7 ~/DCvenv

Activate or retreat from virtual environment.

    $ source ~/DCvenv/bin/activate # activate virtual environment
    $ deactivate # retreat from virtual environment

### Requirements

In the current realse, support is provided for a baselone setup with UR10e, HandE, and RealSense D435. The depenences of DeepClaw are showed below:

- python-pip
- install numpy==1.16.2
- opencv-python==3.3.1.11
- scipy==1.2.2
- tensorflow==1.12.0
- open3d
- RealSense SDK (https://www.intelrealsense.com/developers/), pyrealsense2

## Code Organization

The DeepClaw code is organized as follows:

    configs/                configuration for robotic station for manipulation tasks.
    deepclaw/drivers/       drivers for various robotic hardware, i.e. ur, franka, aubo.
    deepclaw/models/        model zoo for segmentation, classification, pick planning, and motion planning.
    deepclaw/utils/         server setup with dockers and client setup for laptops (x86) and jetson (arm).
    projects/proj_trashSort a sample project to run deepclaw for sorting trash.
    datasets/trash          description of trash sorting dataset
    docs/                   description of this document as a manual.

TODO list in the next update:

    projects/proj_claw      a sample project to run deepclaw in arcade claw game.
    projects/proj_jigsaw    a sample project to run deepclaw in jigsaw game.
    projects/proj_oxTTT     a sample project to run deepclaw in tic-tac-toe game.
    datasets/toys           description of the toy dataset
    datasets/jigsaw         description of jigsaw game pieces dataset
    datasets/mnist          description of mnist dataset

## Bibliography

arXiv