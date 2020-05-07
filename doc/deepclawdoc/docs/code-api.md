# API Reference

The deepclaw library API consists of following parts:

- Hardware Driver API
    - Arm

    - Hand

    - Camera

- Modules Pool API
    - Segmentation

    - Recognition

    - Grasp Planning

    - Motion Planning

- utils API

## Hardware Driver API

The Hardware Driver API is used for controling the Hardware.

### Arm

#### UR10e

> **class**  deepclaw.driver.arms.**UR10eController**(robot_configuration_file_path)

- parameters[in]: robot_configuration_file_path, the robot configuration file, and the file format is yaml.

- return: an instance of UR10e controller class

  _**Functions**_

  ```yaml
  go_home()     
  Description: move to pre-defined joints, the home joints is defined in robot configuration file.
  ```

  ```yaml
  move_j(joints_angle, velocity=None, acceleration=None, solution_space='Joint')   
  Description: go to the target joints positions    
  param[in]:
  - joints_angle: target joint positions of each joints [rad];    
  - velocity: joint acceleration of leading axis [rad/s];   
  - accelerate: joint speed of leading axis [rad/s^2];   
  - solution_space: move style, 'Joint' means it linear in joint-space(inverse kinematics is used to calculate the corresponding joints), and 'Space' means linear in tool-space   
  return: bool, reaching target or not
  ```

  ```yaml
  move_p(position, velocity=None, acceleration=None,solution_space='Joint')   
  Description: go to the target pose(Rotation vector)    
  param[in]:
  - position: target pose;    
  - velocity: joint acceleration of leading axis [rad/s^2];   
  - accelerate: joint speed of leading axis [rad/s];   
  - solution_space: move style, 'Joint' means it linear in joint-space,and 'Space' means linear in tool-space(forward kinematics is used to calculate the corresponding pose)    
  return: bool, reaching target or not
  ```

  ```yaml
  get_state()   
  Description: get robot state    
  return: dictionary, the whole states of the UR10e
  ```

  ```yaml
  verify_state(variable_name, target_value, error=0.0001, time_out=10)   
  Description: verify the robot reaching the target pose(joint or cartesian) or not    
  param[in]:
  - variable_name: target style, joints('q_actual') or cartesian('tool_vector_actual');    
  - target_value: target values;   
  - error: threshold,if the difference between current state and target state is small than threshold, we say the robot reached the target;   
  - time_out: Max time of the motion, [second]    
  return: bool, reaching target or not
  ```

### Hand
#### HandE
> **class**  deepclaw.driver.grippers.handE_controller.**HandEController**(robot_ip = "192.168.1.10",port = 30003)

- parameters[in]: robot_ip, the UR ip which the gripper mounted on; port, the UR ip.

- return: an instance of gripper controller class

  _**Functions**_

  ```yaml
  close_gripper()     
  Description: close the gripper.
  ```

  ```yaml
  open_gripper()     
  Description: open the gripper.
  ```



## Modules Pool API

### Calibration

#### deepclaw.modules.calibration.EyeOnBase.Calibration

*class* deepclaw.modules.calibration.EyeOnBase.**Calibration**(arm, camera, configuration_file)

`Parameters`

- arm
- camera
- configuration_file

`Methods`

- [run]([])(self): Calibrate robot arm and camera

### End-to-End

#### deepclaw.modules.end2end.effecientdet.efficientdet_predictor.efficientdet

*class* deepclaw.modules.end2end.effecientdet.efficientdet_predictor.**efficientdet**(compound_coef=0, weight_path=None, num_classes=204)

`Parameters`

- compound_coef
- weight_path
- num_classes

`Methods`

- [run]([])(self, image_np)
- [display]([])(self, preds, imgs, save_path, imshow=False, imwrite=True)

### Segmentation

#### deepclaw.modules.segmentation.ContourDetector.ContourDetector

*class* **deepclaw.modules.segmentation.ContourDetector.ContourDetector**(mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_NONE, binary_threshold=127, area_threshold=(80, 200), with_angle=False)

```yaml
Parameter
  - mode
  - method
  - binary_threshold
  - area_threshold
  - with_angle
```


`Methods`

- [run]([])(self, color_image)
