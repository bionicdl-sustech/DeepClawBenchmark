====
Pick Planning -- Grasp Pose Detection (GPD)
====
Grasp Pose Detection (GPD) is a method to estimate the position and orientation (6D pose) of an object to be grasped in dense clutter.
This algorithm can predicts grasping poses of a 2-finger parallel gripper in point clouds witout object's 3d models.

----
Overview
----
The whole procedue of the algorithm is showed in Fig.1. The GPD algorithm mainly consists of 2 part: generate a grasp pose candidate; select or score a pose.

.. .. figure:: _static/DeepClawOverview.png
    :align: center
    :figclass: align-center

.. figure:: ./figure-GPD-overview.PNG
  :scale: 30 %
  :alt: alternate text
  :align: center
  :figclass: align-center
  
  Figure 1. Overview of the GPD Algorithm

A qiuck start

 | ./detect_grasps ../cfg/eigen_params.cfg ../tutorials/krylon.pcd
 
&&&&
Grasp Pose Generator
&&&&


 |pose generation API: gpd::GraspDetector::detectGrasps.generateGraspCandidateSets(cloud) in src/detect_grasps.cpp


Before the algorithm starting, we need **preprocess** the point cloud such as denoising, subsampling, segmentaion .etc.

 | API: CandidatesGenerator::preprocessPointCloud in /src/gpd/candidate/candidates_generator.cpp  
 | the implement is in src/gpd/util/cloud.cpp
 
Then we **uniformly randomly simple** the grasp candidators in the point cloud. In each point p, we calculate a darboux frame (F(p)), and generate a pose with the closing plane of the hand is parallel to the cutting plane at p.

 | the simpling is implemented in the preprocessing as subsamples.
 | the API of F(p) is LocalFrame* FrameEstimator::calculateFrame, implemented in src/gpd/candidate/local_frame.cpp


.. figure:: ./figure-GPD-F(p).PNG
  :scale: 40 %
  :alt: alternate text
  :align: center
  :figclass: align-center
  
  Figure2. (a) **Cutting plane**:the plane orthogonal to the direction of minimum principal curvature at point p; (b) Darboux frame (**F(p)**): a surface normal and two principal curvatures.

In each point, we generate a lot of candidates by searching two dimensional grid in Y axis and Θ axis (Yaw angle in the cutting plane) of the F(p) .

 | API: hand_set_list = HandSearch::searchHands.evalHands  in /src/gpd/candidate/hand_search.cpp
 | the implement is in /src/gpd/candidate/hand_set.cpp


&&&&
Candidates Evaluation
&&&&
To reduce the number of hand candidates, the followed 2 constraints are used:

 * The body of the hand is not in collision with the point cloud when the fingers are fully open.
 * The hand closing plane contains p,  and closing plane is a section of the closing plane.

 | the API is HandSearch::reevaluateHypothesis in /src/gpd/candidate/hand_search.cpp

&&&&
Candidates Classification
&&&&
In this part, a CNN (LeNet) method is used to classify the candites. As the input of a CNN is images, we first project the point clouds into images.

 | API: gpd::GraspDetector::detectGrasps.createImages in src/detect_grasps.cpp
 
Then, score the projects and choose the highest one

 | API: gpd::GraspDetector::detectGrasps.createImages.classifyImages(images) in src/detect_grasps.cpp
 | API: gpd::GraspDetector::detectGrasps.createImages.selectGrasps in src/detect_grasps.cpp
 
 
----
Additional Resources
----
ten Pas A, Gualtieri M, Saenko K, et al. Grasp pose detection in point clouds[J]. The International Journal of Robotics Research, 2017, 36(13-14): 1455-1473.

https://github.com/atenpas/gpd
