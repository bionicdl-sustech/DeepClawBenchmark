Example: Claw Machine
=====================

.. figure:: _static/deepclaw-framework.png
    :align: center
    :figclass: align-center


Tasks Description
-----------------
Description of this task family, such as name of tasks, organization of tasks and so on.

Configuration
--------------
 #. Manipulation Environment Description
      * Workspace size
      * Functions of regions
      * Constrains
      * ...

 #. Objects Information
      * List of objects and their descriptions
      * Initial poses of objects
      * ...

 #. Robots/Hardware Setup
      * UR5
      * Onrobot RG6
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

...
