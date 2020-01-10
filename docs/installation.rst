Installation on Linux
=====================

This chapter describes how to install ``DeepClaw`` from source.
DeepClaw framework has only been tested with Python 2.7 and Ubuntu 16.04 LTS.

.. note::
   The following instructions are exemplary for Ubuntu 16.04 LTS system and `Python 2.7`.
   They only work in the supported environments.

Virtual Environment
-------------------
We recommend using a virtual environment (such as virtualenv) to manage DeepClaw.

Install virtualenv.

.. code-block:: shell
    :linenos:

    $ pip install -U virtualenv

Create a new virtual environment.

.. code-block:: shell
    :linenos:

    $ virtualenv -p /usr/bin/python2.7 ~/DCvenv

Activate or retreat from virtual environment.

.. code-block:: shell
    :linenos:

    $ source ~/DCvenv/bin/activate # activate virtual environment
    $ deactivate # retreat from virtual environment

Requirements
------------
The supported hardware of this framework are franka panda, ur10e, ur5; Robotiq handE, RG6, suction cup, Franka gripper; realsense, Kinect Azure, PhotoNeo M. As franka needs a realtime kernel, if you use a franka, you need install a realtime kernel and libfranka, and the details are showed in https://frankaemika.github.io/docs/libfranka.html. 
The depenences of DeepClaw are showed below:

 * ROS http://wiki.ros.org/kinetic/Installation/Ubuntu
 * python-pip
 * install numpy==1.16.2
 * opencv-python==3.3.1.11
 * scipy==1.2.2
 * tensorflow==1.12.0
 * open3d
 * RealSense SDK (https://www.intelrealsense.com/developers/), pyrealsense2




Notes
-----
 * RG6 Driver: uncheck the ``Enable RG`` box under Installation/RG Configuration tab in UR5's teach pendent.
 
