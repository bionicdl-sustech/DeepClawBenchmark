# Calibration
calibration is a method to calculate  the relationship between the sensor and the gripper. There are three formulations [1]:
 * **Hand–eye calibration**: Fig.1 (a)
 * **Robot–world and tool–flange calibration**: Fig.1 (b)
 * **robot–robot calibration**: Fig.1 (c)

 <p align="center"><img src="./calibration.PNG" width="80%"/></p>
 <p align="center">Figure 1. (a) Hand–eye calibration that can be formulated as an <b>AX = XB</b> problem. (b) Robot–world and tool–flange calibration problem that can be formulated to an <b>AX = YB</b> matrix equation. (c) Hand–eye, tool–flange, and robot–robot calibration problem that can be formulated in a matrix equation as <b>AXB = YCZ</b>. [1]</p>
## Eye On Base Implementation
### Configuration Template

```yaml
initial_position:
  - 0.35
  - -0.55
  - 0.25
  - 3.14
  - 0
  - 0
# the numebr of sampled points is cube_size*cube_size*cube_size
cube_size: 4
x_stride: 0.05
y_stride: 0.05
z_stride: 0.02
CALIBRATION_DIR: "/data/calibration_data/xxx-xxx.npz"
OFFSET: 0.16

```



## References
[1] Wu L, Wang J, Qi L, et al. Simultaneous Hand–Eye, Tool–Flange, and Robot–Robot Calibration for Comanipulation by Solving the AXB= YCZ Problem[J]. IEEE TRansactions on robotics, 2016, 32(2): 413-428.