# DeepClawBenchmark

paper | poster | video

Establishing a reproducible and shareable benchmarking for dexterous manipulation has been a significant challenge since the diversity of robot systems, the complexity of manipulation tasks, and a wide selection of metrics. To reduce the entry barrier, we propose **DeepClaw** - a standardized dexterous manipulation protocol, which comprises four common operations to streamline the manipulation process: *localization*, *recognition*, *grasp planning*, and *motion planning*. 

Robot can learning skills that applicable for the similar tasks, called the *task familiy*[1]. We have implemented several manipulation tasks in three task families representing assembly tasks, reasoning tasks and bin-picking tasks separately.

For a detailed decription of DeepClaw and benchmarking tasks, please visit the [website of DeepClaw](https://bionicdl-sustech.github.io/DeepClawBenchmark/)

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
- **franka**: FRANKA arm support.
- **aubo**: AUBO arm support (update later).
- **denso**: DENSO Cobotta arm support (update later).

## <a name="tasks">Tasks</a>
We have implemented some task families with DeepClaw:
- Task Family 1: Jigsaw puzzle
- Task Family 2: Tictactoe Game
- Task Family 3: Claw Machine

## References
[1] O. Kroemer, S. Niekum, and G. Konidaris, “A review of robot learning for manipulation: Challenges, representations, and algorithms,”arXiv preprintarXiv:1907.03146, 2019.
