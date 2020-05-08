# Installation

The DeepClaw is a benchmarking module pool that functions as a Reconfigurable Robotic Manipulation System for Robot Learning. The main homepage can be found [here]( https://bionicdl-sustech.github.io/DeepClawBenchmark/ ). This is the GitHub repository of DeepClaw source code, including instructions for installing and using DeepClaw.

## Requirements

In the current release, support is provided for a baseline setup with UR10e, HandE, and RealSense D435. The dependencies of DeepClaw are showed below:

- RealSense SDK ([how to install]([https://www.intelrealsense.com/developers/]))
  - Python packages
    - numpy
    - PyYAML
    - pyrealsense2

Before using DeepClaw, please ensure you have installed all above packages. The python packages `numpy`, `PyYaml`, and `pyrealsense2` will be installed when isntalling DeepClaw using pip.

## Install using pip

Install DeepClaw using pip in virtual environment.

```
python3 -m pip install DeepClaw
```
## Install from source

Clone repository from Github.

```
git clone https://github.com/bionicdl-sustech/DeepClawBenchmark.git
cd DeepClawBenchmark
```

Install DeepClaw.

```
python3 setup.py install
```
