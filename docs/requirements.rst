Minimum system setup
=======================================

This page specifies the minimum system setup for running the DeepClaw benchmark.
Additional requirements are specified in the documents of each example task.

+----------------------------------------------------------+
| Minimum System                                           |
+===================+======================================+
| Computer          | With Ubuntu 16.04, python3           |
+-------------------+--------------------------------------+
| Robot arm         | proving basic control API like movej |
+-------------------+--------------------------------------+
| End-effector      | Multi-finger gripper or suction cup  |
+-------------------+--------------------------------------+
| camera            | RGB, depth map, point cloud          |
+-------------------+--------------------------------------+

The DeepClaw provides a convenient script for hand-eye calibration under /modules/calibration. The method use a printed chessboard attached to the robot arms.
