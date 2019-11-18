# Driver

In this part, we provide widely used hardware controllers such  as camera sensors, collaborating robot arms and adaptive robot grippers. You can create your own controller by inheriting the parent class.

## Driver of Sensors
### Camera
Basic Functions
- get_frame()
- get_intrinsics()

Extended Functions
- set typical parameters

### Force Sensor
xxxxx

## Driver of Arms
Basic Functions
- move_j(joint, velocity, accelerate, solution_space)
- move_p(position, velocity, accelerate, solution_space)
- get_state()
- verify_state(variable_name, target_value, error=0.02)

Extended Functions
- request specific information
- ...

## Driver of Gripper
Basic Functions
- set_state(variable, value)
- get_state()

Extended Functions
- request specific command

