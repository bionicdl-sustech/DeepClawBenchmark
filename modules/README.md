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

- Localization

```
INPUTS: {ColorImage, DepthImage, PointCloud}
OUTPUTS: {BoundingBox, Mask}
```

- Recognition

```
INPUTS: {BoundingBox, Mask}
OUTPUTS: {Label, Probability}
```

- Grasp Planning

```
INPUTS: {Label, Probability, Constrain}
OUTPUTS: {GraspPose}
```

- Motion Planning

```
INPUTS: {GraspPose}
OUTPUTS: {PointsList}
```

- Execution

```
INPUTS: {PointsList, HardwareState}
OUTPUTS: {Feedback, HardwareState}
```

## Module Design Template

- Localization
  
  ```Python
  modules.localization.__name_of_your_algorithm__ (color_image=None, 
  depth_image=None, point_cloud=None)
  ```

| Parameters                                                        |                                                                     |
| ----------------------------------------------------------------- | ------------------------------------------------------------------- |
| **color_image: *array of shape (width, height, 3)， optional***    | **RGB image array, sampled from RGB camera sensor.**                |
| **depth_image: *array of shape (width, height, 1), optional***    | **Depth image array, provide depth information at each pixel.**     |
| **point_cloud: *array of shape (number of points, 3), optional*** | **Point clould information, each point represented by  (x, y, z).** |

- Recognition
  
  ```python
  modules.recognition.__name_of_your_algorithm__ (color_image=None,
   depth_image=None, point_cloud=None)
  ```
  
  


