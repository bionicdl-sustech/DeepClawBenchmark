Overview
========

.. figure:: _static/DeepClawOverview.png
    :align: center
    :figclass: align-center


    Schematic overview.

The DeepClaw benchmark is a framework for establishing a reproducible and shareable benchmarking for dexterous manipulation.
DeepClaw benchmark provides a standardized dexterous manipulation pipeline consisting of four functionalities: **localization, recognition, grasp planning, and motion planning**.
It also provide necessary components to benchmark manipulations including hardware drivers, data I/O utilities, baseline algorithm modules and evaluation metrics.

The DeepClaw has been used extensively to benchmark a series of manipulation tasks including **claw machine**, **jigsaw game** and **Tic-Tac-Toe**. The source codes of these experiments
are placed under /examples.

.. _drivers:

Drivers
-------
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
    The driver modules provides a base python class for each type of hardware. The functionality of each type have been standardized.
    The users are encouraged to follow the `guidelines <https://github.com/bionicdl-sustech/DeepClawBenchmark/blob/master/docs/_static/Driver_functionaity_requirement.docx>`_ to add new hardwares.

.. _baseline-algorithms:

Modular functionality
---------------------
+---------------------------------------------------------+
| Baseline algorithms for manipulation pipeline           |
+===================+=====================================+
| Localization      | Image edge detection,               |
|                   | DBSCAN[2]                           |
+-------------------+-------------------------------------+
| Recognition       | SIFT feature detection, AlexNet     |
+-------------------+-------------------------------------+
| Grasp planning    | Centroid and Principle axis         |
+-------------------+-------------------------------------+
| End-to-end methods| SSD, Maskrcnn, SD-maskrcnn[3],      |
|                   | Fully convolutional AlexNet         |
+-------------------+-------------------------------------+

Task pipeline
-------------
A benchmark task in DeepClaw is decomposed into repetitive subtasks and each subtask is defined by a pipeline of functionality modules as shown below.

.. figure:: _static/fig-FunctionPipeline.png
    :align: center
    :figclass: align-center


    Pipeline of a task.

Examples
--------
Robot can learning skills that applicable for the similar tasks, called the task familiy[1].
We have implemented several manipulation tasks in three task families representing assembly tasks, reasoning tasks and bin-picking tasks separately.
Please refer to the website for each example for more details.

[1] O. Kroemer, S. Niekum, and G. Konidaris, “A review of robot learning for manipulation: Challenges, representations, and algorithms,”arXiv preprintarXiv:1907.03146, 2019.

[2] Ester, Martin, et al. "A density-based algorithm for discovering clusters in large spatial databases with noise." Kdd. Vol. 96. No. 34. 1996.

[3] Danielczuk, Michael, et al. "Segmenting unknown 3d objects from real depth images using mask r-cnn trained on synthetic data." 2019 International Conference on Robotics and Automation (ICRA). IEEE, 2019.
