# Hierarchical Manipulation Tasks

DeepClaw benchmarking has streamlined the manipulation process into four stages: *localization*, *identification*, *multiple points motion planning*, and *execution*. In this part, we propose a algorithm modules pool corresponding to each stage which has standardized input and output.

## Algorithm Modules Pool

### Calibration

- 2D
- 3D

### Localization

- Segmentation
- Point Cloud Cluster

### Recognition

- Classification

### Grasp Planning

- 2D Pose Estimation
- 3D Pose Estimation
- Min-max Strategy
- A-star Algorithm

### Motion Planning

- xxxx

### End-to-end

- Semantic Segmentation
- Detection

## Standardized Stages Inputs/Outputs

# 

- Localization
  
  ```python
  modules.localization.__name_of_your_algorithm__ (color_image=None, 
  depth_image=None, point_cloud=None)
  ```

| Parameters                                                                 |                                                                                          |
| -------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| **color_image:** *array of shape (width, height, 3)， optional*             | RGB image array, sampled from RGB camera sensor.                                         |
| **depth_image:** *array of shape (width, height, 1), optional*             | Depth image array, provide depth information at each pixel.                              |
| **point_cloud:** *array of shape (number of points, 3), optional*          | Point clould information, each point represented by  (x, y, z).                          |
| **Returns**                                                                |                                                                                          |
| **bounding_box:** *array of shape (2, 2) or array of shape (3, 2)*         | 2D or 3D bounding box information, include coordinate of left top point and size of box. |
| **mask:** *array of shape (n_points, 1) or array of shape (width, height)* | Distinguish each point by giving index                                                   |
| **centers:** *array of shape (n_objects, 3)*                               | Center of objects.                                                                       |

- Recognition
  
  ```python
  modules.recognition.__name_of_your_algorithm__ (color_image=None,
   depth_image=None, point_cloud=None, bounding_box=None,
   mask=None, centers=None)
  ```

| Parameters                                                                                           |                                                                                          |
| ---------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| **bounding_box:**  *array of shape (2, 2) or array of shape (3, 3)*                                  | 2D or 3D bounding box information, include coordinate of left top point and size of box. |
| **mask:**  *array of shape (n_points, 1) or array of shape (width, height)*                          | Distinguish each point by giving index                                                   |
| **centers:**  *array of shape (n_objects, 3)*                                                        | Center of objects.                                                                       |
| **Returns**                                                                                          |                                                                                          |
| **labels:** *array of shape (n_points, 1) or array of shape (width, height)*                         | Distinguish each point by label                                                          |
| **probabilities:** *array of shape (n_points, n_labels) or array of shape (width, height, n_labels)* | Probability of each label in point.                                                      |

- Grasp Planning
  
  ```python
  modules.grasp_planning.__name_of_your_algorithm__ (bounding_box=None, mask=None, centers=None, labels=None, probability=None)
  ```

| Parameters                                                                                           |                                                                                          |
| ---------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| **bounding_box:**  *array of shape (2, 2) or array of shape (3, 3)*                                  | 2D or 3D bounding box information, include coordinate of left top point and size of box. |
| **mask:**  *array of shape (n_points, 1) or array of shape (width, height)*                          | Distinguish each point by giving index                                                   |
| **centers:**  *array of shape (n_objects, 3)*                                                        | Center of objects.                                                                       |
| **labels:** *array of shape (n_points, 1) or array of shape (width, height)*                         | Distinguish each point by label                                                          |
| **probabilities:** *array of shape (n_points, n_labels) or array of shape (width, height, n_labels)* | Probability of each label in point.                                                      |
| **Returns**                                                                                          |                                                                                          |
| **grasp_pose:** *list of length 6*                                                                   | Provide grasp pose information, includes x, y, z, roll, pitch, and yaw.                  |

- Motion Planning
  
  ```python
  modules.recognition.__name_of_your_algorithm__ (grasp_pose=None, constrain=None)
  ```

| Parameters                                      |                                                                         |
| ----------------------------------------------- | ----------------------------------------------------------------------- |
| **grasp_pose:**  *list of length 6*             | Provide grasp pose information, includes x, y, z, roll, pitch, and yaw. |
| constrain:                                      |                                                                         |
| **Returns**                                     |                                                                         |
| **points_list:** *array of shape (n_points, 3)* | Set of points in moving path.                                           |

- Execution
  
  ```python
  modules.recognition.__name_of_your_algorithm__ (points_list=None, hardware_state=None)
  ```

| Parameters                                       |                               |
| ------------------------------------------------ | ----------------------------- |
| **points_list:**  *array of shape (n_points, 3)* | Set of points in moving path. |
| hardware_state:                                  |                               |
| **Returns**                                      |                               |
| feed_back:                                       |                               |
| hardware_state:                                  |                               |
