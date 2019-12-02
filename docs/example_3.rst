Example: Jigsaw Puzzle
==================

.. figure:: _static/deepclaw-framew
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
      * Robot arm
      * Robot gripper
      * Computing platform
      * ...

Procedure
---------

 #. Task1
      Crucial parameters: P1, P2, ...
      Pseudo-code::

         task_display(p1, p2, ...):
           input_0 = [p1, p2, ...]
           main_loop:
             results = subtask_display(input_0)
             metrics_record(results)
          return
        subtask_display(input_0):
          results_0 = localization(input_0)
          results_1 = identification(results_0)
          results_2 = multiple_points_motion_planning(results_1)
          results_3 = execution(results_2)
          return results_3


 #. Task2
      Crucial parameters: P1, P2, ...

Metrics
-------

 #. Experiment 1
      Table of results.

 #. Experiment 2
      Table of results.
