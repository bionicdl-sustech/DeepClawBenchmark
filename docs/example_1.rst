Overview
========

.. figure:: _static/deepclaw-framework.png
    :align: center
    :figclass: align-center


    Schematic overview.

The DeepClaw benchmark is a framework for establishing a reproducible and shareable benchmarking for dexterous manipulation.
DeepClaw benchmark provides a standardized dexterous manipulation pipeline consisting of four subtasks: **localization, recognition, grasp planning, and motion planning**.
It also provide necessary components to benchmark manipulations including hardware drivers, data I/O utilities, baseline algorithm modules and evaluations metrics.

The DeepClaw has been used extensively to benchmark a series of manipulation tasks including claw machine, jigsaw game and TicTacToe. The source codes of these experiments
are placed under /examples.

.. _drivers:
Drivers
--------------
+---------------------------------------------------------+
| Supporting Hardwares                                    |
+===================+=====================================+
| Robot arms        | UR5, UR10, Franka Panda             |
+-------------------+-------------------------------------+
| Grippers          | RG6, Robotiq HandE                  |
+-------------------+-------------------------------------+
| Cameras           | Realsense, Azure Kinect, Photoneo   |
+-------------------+-------------------------------------+

.. hint::
    The best performance can be achieved when connecting directly to the LAN port of Control.
    This requires setting up a static IP for the shop floor network in the administrator's interface
    beforehand. See :ref:`setting-up-the-network`.

.. _baseline-algorithms:
Baseline algorithms
--------------
+---------------------------------------------------------+
| Baseline algorithms for manipulation pipeline           |
+===================+=====================================+
| localization      | UR5, UR10, Franka Panda             |
+-------------------+-------------------------------------+
| recognition       | RG6, Robotiq HandE                  |
+-------------------+-------------------------------------+
| grasp planning    | Realsense, Azure Kinect, Photoneo   |
+-------------------+-------------------------------------+
