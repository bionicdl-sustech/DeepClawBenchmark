# DeepClawBenchmark

paper | poster | video

The DeepClaw benchmark is a framework for establishing a reproducible and shareable benchmarking for dexterous manipulation. DeepClaw benchmark provides a standardized dexterous manipulation pipeline consisting of four subtasks: localization, recognition, grasp planning, and motion planning. It also provide necessary components to benchmark manipulations including hardware drivers, data I/O utilities, baseline algorithm modules and evaluation metrics.

The DeepClaw has been used extensively to benchmark a series of manipulation tasks including claw machine, jigsaw game and TicTacToe. The source codes of these experiments are placed under /examples.
![](https://github.com/bionicdl-sustech/DeepClawBenchmark/blob/master/Documents/Figs/deepclaw-framework.png)

## Quick Start

### Prerequisites

DeepClaw framework has only been tested with *Python 2.7* and *Ubuntu 16.04 LTS*. We recommend using a virtual environment (such as virtualenv) to manage DeepClaw.

Install virtualenv.

```shell
$ pip install -U virtualenv
```

Create a new virtual environment.

```shell
$ virtualenv -p /usr/bin/python2.7 ./DCvenv
```

Activate or retreat from virtual environment.

```shell
$ source ./DCvenv/bin/activate # activate virtual environment
$ deactivate # retreat from virtual environment
```

### Installation

Clone or download DeepClaw from Github.

```shell
$ git clone https://github.com/bionicdl-sustech/DeepClawBenchmark.git
$ cd ./DeepClawBenchmark
```

Install Prerequisites:

```shell
$ pip install -r requirements.txt
```
Build libfranka server
```shell
$ cd ./DeepClawBenchmark/driver/arms/Franka/libfraka_server
$ mkdir build
$ cd build
$ cmake ..
$ make
```
### Verify Installation
Run calibration task with your drivers, for example, UR10e, HandE, Kinect and so on.

```shell
$ python main.py ur10e hande kinect-azure calibration true
```

There also are some test cases for testing your installation and calibration.

[Test cases](https://github.com/bionicdl-sustech/DeepClawBenchmark/blob/master/Documents/TestCases.md)

## <a name="tasks">Tasks</a>
We have implemented some tasks using DeepClaw with classical algorithm modules:
- Task Family 1: [Jigsaw](https://github.com/bionicdl-sustech/DeepClawBenchmark/blob/master/Documents/Jigsaw_task/task_description.md)
- Task Family 2: Tic-tac-toe Game
- Task Family 3: Toy Claw Machine

Find the task description template [here](https://github.com/bionicdl-sustech/DeepClawBenchmark/blob/master/Documents/Task-Description-Template.md).
And we encourage developers to create new tasks ([how to create](https://github.com/bionicdl-sustech/DeepClawBenchmark/blob/python2.7/documents/How-to-Create-Task.md)).
## Algorithm Modules
We also provide modules pool for developers to assembly their own manipulation tasks.

Find all modules description [here](https://github.com/bionicdl-sustech/DeepClawBenchmark/tree/python2.7/modules).
And how to create a new module.
## References
[1] O. Kroemer, S. Niekum, and G. Konidaris, “A review of robot learning for manipulation: Challenges, representations, and algorithms,”arXiv preprintarXiv:1907.03146, 2019.
