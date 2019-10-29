# Hierarchical Manipulation Tasks

DeepClaw benchmarking has streamlined the manipulation process into four stages: *localization*, *identification*, *multiple points motion planning*, and *execution*. In this part, we propose a algorithm modules pool corresponding to each stage which has standardized input and output.

## Algorithm Modules Pool

### Calibration

- 2D
- 3D

### Localization

- Segmentation
- xxxxx

### Identification

- Classification
- xxxx

### Multiple Points Motion Planning

- Min-max Strategy
- A-star Algorithm
- xxxx

### End-to-end

- Deep Neural Network
- Semantic Segmentation
- Detection

## Standardized Stages Inputs/Outputs

- Localization

```
INPUTS:
OUTPUTS:
```

- Identification

```
INPUTS:
OUTPUTS:
```

- Multiple Points Motion Planning

```
INPUTS:
OUTPUTS:
```

- Execution

```
INPUTS:
OUTPUTS:
```

## Module Design Template

```
def module(input):
	line 1
	line 2
	...
	return output
```

