# Hierarchical Manipulation Tasks

DeepClaw benchmarking has streamlined the manipulation process into four stages: *localization*, *identification*, *multiple points motion planning*, and *execution*. In this part, we propose a algorithm modules pool corresponding to each stage which has standardized input and output.

## Algorithm Modules Pool

### Calibration

- 2D
- 3D

### Localization

- Segmentation
- xxxxx

### Recognition

- Classification
- xxxx

### Grasp Planning

- Pose Estimation

### Motion Planning

- Min-max Strategy
- A-star Algorithm
- xxxx

### End-to-end

- Deep Neural Network
- Semantic Segmentation
- Detection

## Standardized Stages Inputs/Outputs

- Segmentation

```
INPUTS: {ColorImage, DepthImage, PointCloud}
OUTPUTS: {ColorImage, DepthImage, PointCloud, BoundingBox, SegmantedMask}
```

- Recognition

```
INPUTS: {ColorImage, DepthImage, PointCloud, BoundingBox, SegmentedMask}
OUTPUTS: {BoundingBox, Label, Probability}
```

- Grasp Planning

```
INPUTS: {BoundingBox, Label, Probability, Constrain}
OUTPUTS: {PointsList}
```

- Motion Planning

```
INPUTS: {PointsList, HardwareState}
OUTPUTS: {HardwareState}
```

- Execution

```
INPUTS: {PointsList, HardwareState}
OUTPUTS: {HardwareState}
```

## Module Design Template

```
def module(input):
	line 1
	line 2
	...
	return output
```

