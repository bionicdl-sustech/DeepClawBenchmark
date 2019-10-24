# DeepClawBenchmark

paper | poster | video

Establishing a reproducible and shareable benchmarking for dexterous manipulation has been a significant challenge since the diversity of robot systems, the complexity of manipulation tasks, and a wide selection of metrics. To reduce the entry barrier, we propose **DeepClaw** - a standardized dexterous manipulation protocol, which comprises four common operations to streamline the manipulation process: *localization*, *identification*, *multiple points motion planning*, and *execution*. We implement three classical manipulation tasks following DeepClaw protocol, you can find them <a href="#tasks">here</a>. In addition, we propose metrics measuring above operations in two aspects: spatial and temporal reasoning.

![](https://github.com/bionicdl-sustech/DeepClawBenchmark/blob/master/Documents/Figs/deepclaw-framework.png)

## Quick Start

### Prerequisites

DeepClaw framework has only been tested with *Python 2.7* and *Ubuntu 16.04 LTS*. We recommend using a virtual environment (such as virtualenv) to manage DeepClaw.

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
$ sudo sh install.sh realsense ur
```

The brackets indicate optional arguments to switch installation methods.

The first argument specifies the version:

- **realsense**: RealSense D435 support.

The second argument specifies the installation mode:

- **ur**: UNIVERSAL ROBOT arm series support (UR5 and UR10e).
- **franka**: FRANKA arm support (update later).
- **aubo**: AUBO arm support (update later).
- **denso**: DENSO Cobotta arm support (update later).

There are some test cases for testing your installation and calibration.

[Test cases](https://github.com/bionicdl-sustech/DeepClawBenchmark/blob/master/Documents/TestCases.md)

## <a name="tasks">Tasks</a>
We have implemented some task families with DeepClaw:
- Task Family 1: [Jigsaw](https://github.com/bionicdl-sustech/DeepClawBenchmark/blob/master/Documents/Jigsaw_task/task_description.md)
- Task Family 2: Tictactoe Game
- Task Family 3: Toy-Claw