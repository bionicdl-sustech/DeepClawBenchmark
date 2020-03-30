The configuration in this floder is a whole parameters of the hardware. including robot, camere, and gripper.
<!-- Load yaml files-->
An example of loading the config parameters:
```
import yaml
with open('configuration.yaml','r') as f:
    temp = yaml.load(f.read())
    robot_ip = temp['robot']['ip']

```

The units of the parameters is m, rad, m/s, m/s^2, gram, pixel.
