Example: Jigsaw Puzzle
======================

.. figure:: _static/fig-ExpJigsaw-Design.png
    :align: center
    :figclass: align-center

Tasks Description
-----------------
The jigsaw tasks contain 3 tasks, which evaluate the different performance of the robot manipulation.

Configuration
--------------
 #. Manipulation Environment Description
      * Workspace size: 600 mm * 700 mm
      * Functions of regions: the objects are placed on the pick space in the begining, and putted on the place space in the end.
      * Constrains: the face with texture is upside
      * ...

 #. Objects Information
      * List of objects and their descriptions
      * Initial poses of objects
      * ...

 #. Robots/Hardware Setup
      * UR10e, UR5, Franka Panda
      * Suction cup

Metrics
-------
IoU and average precision for segmentation and recognition.

Grasp success rate for grasp planning.

Area ratio for task completion.
