# DeepClawBenchmark

paper | [poster](https://github.com/ancorasir/CobotBenchmark/blob/master/Documents/DeepClaw%20Poster-pre-version.pdf) | video

Establishing a reproducible and shareable benchmarking for dexterous manipulation has been a significant challenge since the diversity of robot systems, the complexity of manipulation tasks, and a wide selection of metrics. To reduce the entry barrier, we propose **DeepClaw**: a standardized dexterous manipulation protocol, which comprises four common operations to streamline the manipulation process: *localization*, *identification*, *multiple points motion planning*, and *execution*. In addition, we propose metrics measuring above operations in two aspects: spatial and temporal reasoning.

![](https://github.com/ancorasir/CobotBenchmark/blob/master/Documents/deepclaw-framework.png)

## Quick Start

### Prerequisites

DeepClaw framework has only been tested with Python 2.7 and Ubuntu 16.04 LTS. We recommend using a virtual environment (such as virtualenv) to manage DeepClaw.

Install virtualenv.

```shell
$ sudo pip install -U virtualenv
```

Create a new virtual environment.

```shell
$ virtualenv --system-site-packages -p python2.7 ./venv
```

Activate or retreat from virtual environment.

```shell
$ source ./venv/bin/activate # activate virtual environment
$ deactivate # retreat from virtual environment
```

### Installation

Clone or download DeepClaw from Github.

```shell
$ git clone https://github.com/bionicdl-sustech/DeepClawBenchmark.git
$ cd ./DeepClawBenchmark
```

Run the DeepClaw installation helper script:

```shell
$ sudo sh install.sh {cpu|gpu} {ur|franka|aubo|denso}
```

The brackets indicate optional arguments to switch installation methods.

The first argument specifies the version:

- **cpu**: no TensorFlow GPU support
- **gpu**: TensorFlow GPU support for Toy-Claw CNN evaluating.

The second argument specifies the installation mode:

- **ur**: UNIVERSAL ROBOT arm series support.
- **franka**: FRANKA arm support.
- **aubo**: AUBO arm support.
- **denso**: DENSO Cobotta arm support.

Test the installation:

```shell
$ python main.py {ur|frank|aubo|denso} test
```

## Calibration

We provide a naive hand-eye calibrating method which only utilizes 2-D dimension information. Calibrating mark could be download here. There are three steps in this hand-eye calibrating method:

1. Move the end-effector of robot arm to four marks step by step, make sure the center point of end-effector matches each center of mark, and record all the position of end-effector center in robot arm coordinate.
2. Fix camera pose, and capture a image frame. Record all position of mark's center in camera coordinate.
3. Update calibration information in main.py.

You can test your calibration:

```shell
$ python main.py {ur|franka|aubo|denso} calibration_test
```

## Examples

Go the following examples to see the basic functionality.

1. Robot controller operations.
2. Camera controller operations.
3. Add a new module (task/controller/function).