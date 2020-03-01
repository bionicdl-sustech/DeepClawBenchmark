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
The whole procedue of the algorithm is showed in Fig.1. The GPD algorithm mainly consists of 2 part: generate a grasp pose candidator; select or score a pose.

.. .. figure:: _static/DeepClawOverview.png
    :align: center
    :figclass: align-center

.. figure:: ./figure-GPD-overview.PNG
  :scale: 30 %
  :alt: alternate text
  :align: center
  :figclass: align-center
  
  Figure 1. Overview of the GPD Algorithm

&&&&
Grasp Pose Generator
 | ./detect_grasps ../cfg/eigen_params.cfg ../tutorials/krylon.pcd

&&&&
Before the algorithm starting, we need **preprocess** the point cloud such as denoising, subsampling, segmentaion .etc.

 | The API is CandidatesGenerator::preprocessPointCloud in /src/gpd/candidate/candidates_generator.cpp  
 | the implement is in src/gpd/util/cloud.cpp
 
Then we **uniformly randomly simple** the grasp candidators in the point cloud. In each point p, we calculate a darboux frame (F(p)), and generate a pose with the closing plane of the hand is parallel to the cutting plane at p.

 | the simpling is implemented in the preprocessing as subsamples.
 |
 | the API of pose candidators generation is std::vector<Grasp> CandidatesGenerator::generateGraspCandidates(const CloudCamera& cloud_cam)
 |

 

.. figure:: ./figure-GPD-F(p).PNG
  :scale: 30 %
  :alt: alternate text
  :align: center
  :figclass: align-center
  
  Figure2. (a) **Cutting plane**:the plane orthogonal to the direction of minimum principal curvature at point p; (b) Darboux frame (**F(p)**): a surface normal and two principal curvatures.

The simple and frame calculation APIs are showed below:  
 | The API is LocalFrame* FrameEstimator::calculateFrame in /src/gpd/candidate/candidates_generator.cpp  
 | the implement is in src/gpd/util/cloud.cpp
 
the
 | the API of hand is std::vector<std::unique_ptr<Hand>> CandidatesGenerator::generateGraspCandidates
  






####################
Additional Resources
####################
ten Pas A, Gualtieri M, Saenko K, et al. Grasp pose detection in point clouds[J]. The International Journal of Robotics Research, 2017, 36(13-14): 1455-1473.

https://github.com/atenpas/gpd
