# Examples

## Examples

### Robot Cell Configuration

The configuration of robot cell is opening, so users need to initialize their own programming environments. We propose configurations employing some widely used robot cells here. Anyone can replace drivers in any system of these robot cells by other divers corresponding to their hardware.

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

In this example, proposed robot cell configuration is:

| Robot Cell Configuration | Group Number         |
| ------------------------ | -------------------- |
| Perception System        | <a href="#PS">#1</a> |
| Decision System          | <a href="#DS">#2</a> |
| Manipulation System      | <a href="#MS">#4</a> |
| Experimental Environment | <a href="#EE">#1</a> |

### Quick Start

1. Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu), [MoveIt!](https://moveit.ros.org/install/), and [ROS Control](http://wiki.ros.org/ros_control).
2. Create a ROS workspace, and clone [denso driver](https://github.com/DENSORobot/denso_cobotta_ros.git).

```shell
$ mkdir -p ~/denso_ws/src
$ cd ~/denso_ws/src
$ git clone https://github.com/DENSORobot/denso_cobotta_ros.git
$ cd ~/denso_ws
$ catkin_make
```

3. Clone [DeepClaw](https://github.com/ancorasir/CobotBenchmark.git).

```shell
$ cd
$ git clone https://github.com/ancorasir/CobotBenchmark.git
$ cd CobotBenchmark
```

4. Run demo.

```shell
$ python main.py
```

Then, image frame captured by D435 will be show in your screen.
