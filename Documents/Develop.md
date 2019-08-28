# CobotBenchmark

paper | [poster](https://github.com/ancorasir/CobotBenchmark/blob/master/Documents/DeepClaw%20Poster-pre-version.pdf) | video

Establishing a reproducible and shareable benchmarking for dexterous manipulation has been a significant challenge since the diversity of robot systems, the complexity of manipulation tasks, and a wide selection of metrics. To reduce the entry barrier, we propose **DeepClaw**: a standardized dexterous manipulation protocol, which comprises four common operations to streamline the manipulation process: *localization*, *identification*, *multiple points motion planning*, and *execution*. In addition, we propose metrics measuring above operations in two aspects: spatial and temporal reasoning.

![](https://github.com/ancorasir/CobotBenchmark/blob/master/Documents/deepclaw-framework.png)

## Quick Start

### Robot Cell Configuration

The configuration of robot cell is opening, so users need to initialize their own programming environments. We propose configurations employing some widely used robot cells here. Anyone can replace drivers in any system of these robot cells by other divers corresponding to their hardware.

#### Configuration of Robot Cell

- <a name="PS">Perception System</a>: 

| #    | Camera                                                       |
| ---- | ------------------------------------------------------------ |
| 1    | [RealSense D435](https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d435.html) |

- <a name="DS">Decision System</a>: 

| #    | CPU                                                          | GPU                                                          | OS                                                |
| ---- | ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------- |
| 1    | [Intel Core i7](https://www.intel.com/content/www/us/en/products/processors/core/i7-processors/i7-7567u.html) | [Intel Iris Plus Graphics 650](https://www.intel.com/content/www/us/en/support/products/98912/graphics-drivers/graphics-for-7th-generation-intel-processors/intel-iris-plus-graphics-650.html) | [Ubuntu 16.04](http://releases.ubuntu.com/16.04/) |
| 2    | [Intel Core i5](https://www.intel.com/content/www/us/en/products/processors/core/i5-processors/i5-8300h.html) | [NVIDIA GeForce GTX 1050 Ti](https://www.geforce.com/hardware/desktop-gpus/geforce-gtx-1050-ti/specifications) | [Ubuntu 16.04](http://releases.ubuntu.com/16.04/) |

- <a name="MS">Manipulation System</a>: 

| #    | Robot Arm                                                    | End-effector                                                 |
| ---- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| 1    | [UNIVERSAL ROBOTS 5 (UR5)](https://www.universal-robots.com/products/ur5-robot) | [RG2 GRIPPER](https://www.universal-robots.com/plus/end-effectors/rg2-gripper/) |
| 2    | [UNIVERSAL ROBOTS 10e (UR10e)](https://www.universal-robots.com/products/ur10-robot/) | [HandE](https://www.universal-robots.com/plus/end-effectors/hand-e/) |
| 3    | [Franka Panda](https://www.franka.de/)                       | Original Fingers                                             |
| 4    | [Denso Cobotta](https://www.denso-wave.com/en/robot/product/collabo/cobotta.html) | Original Fingers                                             |

- <a name="EE">Execution Environments</a>: 

| #    | Workspace Size (L x W x H) | Demo             |
| ---- | -------------------------- | ---------------- |
| 1    | 80mm x 60mm x 40 mm        | picture \| video |

#### Drivers Installation

Install drivers for your hardware.

| Hardware         | Driver                                                      |
| ---------------- | ----------------------------------------------------------- |
| RealSense D435   | [driver](https://pypi.org/project/pyrealsense2/)            |
| UNIVERSAL ROBOTS | [driver](https://github.com/SintefManufacturing/python-urx) |
| Franka Panda     | [driver](https://frankaemika.github.io/)                    |
| Denso Cobotta    | [driver](https://github.com/DENSORobot/denso_cobotta_ros)   |

### Add New Modules

Firstly, clone CobotBenchmark.

```shell
$ git clone https://github.com/ancorasir/CobotBenchmark.git
```

Recommended directory structure for adding new modules:

```
CobotBenchmark/
├── Data
│   └── ...
├── Driver
│   ├── __init__.py
│   ├── Camera
│   ├── Cobotta
│   ├── ...
│   └── UR10e
├── Examples
│   └── ...
├── Functions
│   └── ...
└── ToolKit
    └── ...
```

#### Camera Module

Then fork the "camera_controller" template as your own camera controller.

```python
class RealsenseController(object):
    def __init__(self, width = 1280, hight = 720,fps = 30):
        self.width = width
        self.hight = hight
        self.fps = fps

        #your own configuration here.
        #...

    def getImage(self):
        #your own code here.
        #depth_image = ...
        #color_image = ...
        #para 1 = ...
        #para 2 = ...
        #...
        return color_image,[depth_image,para1,para2,...]
```

#### Robot Arm Module

Or create a robot arm controller following by "robot_controller_template".

```python
class ROBOTARMController:
    def __init__(self, p1=xx, p2=xxx):
        #initial parameters here

    def get_pos(self):
        #your own code here to get arm pose, return a list with 6 elements.
        pose = np.zeros(6)
        pose = [x, y, z, Rx, Ry, Rz]
        return pose

    def goHome(self):
        print('homing...')
        self.movej(HOME_POSE[0], HOME_POSE[1], HOME_POSE[2], 
                   HOME_POSE[3], HOME_POSE[4], HOME_POSE[5])
    
    def movej(self,x, y, z, Rx, Ry, Rz, acceleration=0, velocity=0, useJoint=False):
        if(useJoint==False):
            #move with position command
        else:
            #move with joints state command

    def verifyPostion(self,targetPosition):
        #check if arrived to target position
```

### Monitor and Data Collecting

Waiting for update.

## Examples

1. [Denso Cobotta](https://github.com/ancorasir/CobotBenchmark/blob/master/Documents/Examples.md)