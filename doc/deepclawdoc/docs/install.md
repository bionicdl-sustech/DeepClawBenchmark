# The DeepClaw Benchmark

The DeepClaw is a benchmarking model zoo that functions as a Reconfigurable Robotic Manipulation System for Robot Learning. The main homepage for Julia can be found at [deepclaw.ancorasir.com](https://deepclaw.ancorasir.com/). This is the GitHub repository of DeepClaw source code, including instructions for installing and using DeepClaw, below.

# Quick Start

As of now, DeepClaw framework has been tested with Python 3.7 and Ubuntu 18.04 LTS.

## Pre-requirements

In the current release, support is provided for a baseline setup with UR10e, HandE, and RealSense D435. The dependencies of DeepClaw are showed below:

- RealSense [SDK]([https://www.intelrealsense.com/developers/])

Before using DeepClaw, please ensure you have installed all above packages.

## Install using pip

Install DeepClaw using pip in virtual environment.

    $ python3 -m pip install DeepClaw

## Install using Docker

Update later.

## Install from source

Clone repository from Github.

```
$ git clone https://github.com/bionicdl-sustech/DeepClawBenchmark.git
$ cd DeepClawBenchmark
```

Install DeepClaw.

```
$ python3 setup.py install
```

