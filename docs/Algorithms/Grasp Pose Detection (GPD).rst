====
Pick Planning -- Grasp Pose Detection (GPD)
====
Grasp Pose Detection (GPD) is a method to estimate the position and orientation (6D pose) of an object to be grasped in dense clutter.
This algorithm can predicts grasping poses of a 2-finger parallel gripper in point clouds witout object's 3d models.

####################
Details
####################

----
Overview
----
The whole procedue of the algorithm is showed in Fig.1.


.. .. figure:: _static/DeepClawOverview.png
    :align: center
    :figclass: align-center

.. figure:: ./figure-GPD-overview.PNG
  :scale: 30 %
  :alt: alternate text
  :align: center
  
  Figure 1. Overview of the GPD Algorithm


****
Sample
****

~~~~
S
~~~~




####################
Additional Resources
####################
ten Pas A, Gualtieri M, Saenko K, et al. Grasp pose detection in point clouds[J]. The International Journal of Robotics Research, 2017, 36(13-14): 1455-1473.

https://github.com/atenpas/gpd
