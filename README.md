# DeepClawBenchmark <!-- omit in toc -->

paper | poster | video

Establishing a reproducible and shareable benchmarking for dexterous manipulation has been a significant challenge since the diversity of robot systems, the complexity of manipulation tasks, and a wide selection of metrics. To reduce the entry barrier, we propose **DeepClaw** - a standardized dexterous manipulation protocol, which comprises four common operations to streamline the manipulation process: *localization*, *recognition*, *grasp planning*, and *motion planning*. 

Robot can learning skills that applicable for the similar tasks, called the *task familiy*[1]. We have implemented several manipulation tasksa in three task families representing assembly tasks, reasoning tasks and bin-picking tasks separately.

For a detailed decription of DeepClaw and benchmarking tasks, please visit the [website of DeepClaw](https://bionicdl-sustech.github.io/DeepClawBenchmark/)

![](https://github.com/bionicdl-sustech/DeepClawBenchmark/blob/master/Documents/Figs/deepclaw-framework.png)

# Quick Start <!-- omit in toc -->

To be continued ... (the kind of code that anyone needs to get started with DeepClaw)

# A Few Notes <!-- omit in toc -->

## General Architecture
- Each folder must contain a `Readme.md` to describe the files inside.
- Cloud Robotics Setup in a Local Network **(Better if we can add a picture)**
  - `Com UserX`: Realtime OS system for robot control, i.e. Franka
mainly runs the driver folder under DeepClaw for robot hardware control
Process hardware IO data in the save folder of DeepClaw for processing
All environment configured base on the robot's requirement
  - `Com Server`:  can be a server, i.e. goldenboy, communicate through TCP/IP
With modules folder under DeepClaw for algorithm processing
Process models output in the save folder for data storage and processing
Since each algorithm may require different backbone environment or setting, can use docker to isolate the algorithm environment, read / write data from / in save folder or TCP/IP directly, avoid contaminating the robot control environment settings
  - `Communication between ComUserX and ComServer`: 
    - Different students/users may use different ComUserX, i.e. ComUser1, ComUser2, to connect to separate accounts on ComServer
    - Each account on ComServer is allocated a GPU and docker environment to communicate with its ComUserX
  
## Usage Guideline
- All editing is recommended using Microsoft VScode
- All users are encouraged to use this Github Repository for version control

# Folder Structure <!-- omit in toc -->

- [**config**: robot configuration and experiment environoment](#config-robot-configuration-and-experiment-environoment)
- [**data**: example datasets, usually not changed](#data-example-datasets-usually-not-changed)
- [**docs**: compiled documentation based on all `Readme.MD`](#docs-compiled-documentation-based-on-all-readmemd)
- [**projects**: simple examples of deepclaw for test](#projects-simple-projects-based-on-deepclaw)
- [**modules**: all algorithms used in deepclaw, docker-style](#modules-all-algorithms-used-in-deepclaw-docker-style)

# **config**: robot configuration and experiment environoment

Please refer to the `Readme.MD` inside this folder for further details.

This folder shall include all configuration files and environment settings for each deepclaw station in the lab. This is already reflected in the subfolder structure and there must include a Readme.MD inside each folder to describe its settings, written by the user who set it up in the first place. Such a structure is to avoid contamination of environment settings and conflict of files. You should try using virtual environments whenever you can and explain everything in the markdown file. This folder may be used by both the ComUserX and ComServer.

# **data**: example datasets, usually not changed

Please refer to the `Readme.MD` inside this folder for further details.

This folder shall inclded all kinds of benchmarking datasets that you might need during experiment and validation. If the folders is too large, then you will still create a folder with detaile description of this dataset with a link to access the data within the BionicDL lab. You can contact Prof. Song to set up a OneDrive link for you. Please note that the files inside each dataset inside this folder are not meant to be changed constantly, as they need to be constantly reused for various testing and validation. This folder will be probabaly saved in the ComServer, but will be definitely used in the ComServer.

## File Storage <!-- omit in toc -->
A few options for your reference. 
- OneDrive (sharepoint)
- Baidu Yunpan
- Google Drive (contact Chaoyang)

# **docs**: compiled documentation based on all `Readme.MD`

Please refer to the `Readme.MD` inside this folder for further details.

## Readmes Combined into a Full Documentation <!-- omit in toc -->
This folder shall be simple. Basically, it is a documentation of the DeepClaw, which shall be compiled automatically based on all the Readme files in each folder following its folder structure. 

## How to Merge with Previosu Documentation? <!-- omit in toc -->
This is a minor problem that we will solve piece by piece.

# **driver**: driver files for all hardware and software <!-- omit in toc -->

Please refer to the `Readme.MD` inside this folder for further details.

This folder is a bit hardcore, which includes all drivers and versions of the drivers for each hardware. The subfolders shall be created based on the brand and model of each hardware and software. This folder must be saved in the ComUserX.

# **projects**: simple projects based on deepclaw

Please refer to the `Readme.MD` inside this folder for further details.

## Benchmarking Papers/Codes/Examples <!-- omit in toc -->
This folder shall include all examples, or projects using the config, drivers and modules in DeepClaw. For example, the jigsaw, arcade claw, tic-tac-toe and others, saved in subfolder. This is the place where magic happens, and everything start to work. 

## A Place to Host Your Individual Research <!-- omit in toc -->
Your individual research can be hosted under this folder to manage your research or collbaoration. All papers published shall be hosted under this folder as an example. 

# **modules**: all algorithms used in deepclaw, docker-style

Please refer to the `Readme.MD` inside this folder for further details.

## Model Zoo <!-- omit in toc -->
This folder may be the most challenging one, as this folder will involve all kinds of algorithms that published by others or created/reproduced by members at BionicDL. In another word, this folder can be viewed as a model zoo. This could be very helpful when one wants to test algorithms or the latest research published by anyone. But the challenge would be how we are going to resolve the version control issues with all kinds of backbone and setup. 

## Cloud Robotics in A Local Network (BionicDL) <!-- omit in toc -->
What we plan to do as a potential solution is something similar to Cloud Robotics. This folder will be mainly hosted/maintained at a local server (the goldenboy, or the serbreeze) with sufficient capability to do all the heavy computation. Based on our current capability, we should be able to support as many as 8 users at the same time (8 GPUs in total in both machines). 

## docker-style <!-- omit in toc -->
To avoid conflicts of settings, we plan to adopt dockers to host individual models in each module and also all its derivatives for each user. This is yet a feature we need to test and improve. A potential problem is this folder being too large to update (let's cross the bridge when we see it).


<!-- ## Quick Start

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
- Task Family 1: [Jigsaw puzzle](https://github.com/bionicdl-sustech/DeepClawBenchmark/blob/master/documents/Jigsaw_task/task_description.md)
- Task Family 2: Tictactoe Game
- Task Family 3: Claw Machine

## References
[1] O. Kroemer, S. Niekum, and G. Konidaris, “A review of robot learning for manipulation: Challenges, representations, and algorithms,”arXiv preprintarXiv:1907.03146, 2019. -->

## Notes:
When you update your codes, fetch the branch at first and merge the branch then.
Do not use force pull or force push to update the codes, as it will cover others' or your codes.
