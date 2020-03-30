Example: Claw Machine
=====================

.. figure:: _static/fig-ExpClaw-Design.png
    :align: center
    :figclass: align-center


Tasks Description
-----------------
This is a bin clearing task similar to the claw machine. The robot arm transport soft toys from a bin full of soft toys to another bin.

Configuration
--------------
 #. Manipulation Environment Description
      * A table top in front of the robot base
      * Camera mounted about 1 meter above the table top and looking downward.
      * gripper always kept vertically.

 #. Objects Information
      * The grasping objects in this task are 8 soft toys.
      * At the start of the task, 8 soft toys are randomly place in a shallow bin.

 #. Robots/Hardware Setup
      * UR5
      * Onrobot RG6
      * Realsense 435

Metrics
-------
Time cost of each functionality step and success rate of grasp.
