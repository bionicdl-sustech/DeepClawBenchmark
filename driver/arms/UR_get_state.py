import socket
import struct
import numpy as np
HOST = "192.168.1.10" # The remote host
# the 30003 port is a realtime port, there are no 1108 data from other port
PORT = 30003 # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
ur_msg = s.recv(1108)
ur_msg = ur_msg.encode("hex")
#data decode
Message_Size = struct.unpack('!i', ur_msg[0:8].decode('hex'))[0]
print('Message_Size',Message_Size)
Time = struct.unpack('!d', ur_msg[8:24].decode('hex'))[0]
print('Time',Time)

# joints
q_target = np.zeros(6)
start_mark = 24
size_length = 16
for m in range(6):
    q_target[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('q_target',q_target*180/np.pi)

#joint velocities
qd_target = np.zeros(6)
start_mark = 120
size_length = 16
for m in range(6):
    qd_target[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('qd_target',qd_target*180/np.pi)

#joint accelerations
qdd_target = np.zeros(6)
start_mark = 216
size_length = 16
for m in range(6):
    qdd_target[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('qdd_target',qdd_target*180/np.pi)

#Target joint currents
I_target = np.zeros(6)
start_mark = 312
size_length = 16
for m in range(6):
    I_target[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print("I_target",I_target)

# Target joint moments (torques)
M_target = np.zeros(6)
start_mark = 408
size_length = 16
for m in range(6):
    M_target[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print("M_target",M_target)

# Actual joint positions
q_actual = np.zeros(6)
start_mark = 504
size_length = 16
for m in range(6):
    q_actual[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print("q_actual",q_actual*180/np.pi)

# Actual joint velocities
qd_actual = np.zeros(6)
start_mark = 600
size_length = 16
for m in range(6):
    qd_actual[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('qd_actual',qd_actual*180/np.pi)

# Actual joint currents
I_actual = np.zeros(6)
start_mark = 696
size_length = 16
for m in range(6):
    I_actual[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('I_actual',I_actual)

#Joint control currents
I_control = np.zeros(6)
start_mark = 792
size_length = 16
for m in range(6):
    I_control[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('I_control',I_control)

# Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector
Tool_vector_actual = np.zeros(6)
start_mark = 888
size_length = 16
for m in range(6):
    Tool_vector_actual[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Tool_vector_actual',Tool_vector_actual)

# Actual speed of the tool given in Cartesian coordinates
TCP_speed_actual = np.zeros(6)
start_mark = 984
size_length = 16
for m in range(6):
    TCP_speed_actual[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('TCP_speed_actual',TCP_speed_actual)

#Generalised forces in the TCP
TCP_force = np.zeros(6)
start_mark = 1080
size_length = 16
for m in range(6):
    TCP_force[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('TCP_force',TCP_force)

#Target Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector
Tool_vector_target = np.zeros(6)
start_mark = 1176
size_length = 16
for m in range(6):
    Tool_vector_target[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Tool_vector_target',Tool_vector_target)

#Target speed of the tool given in Cartesian coordinates
TCP_speed_target = np.zeros(6)
start_mark = 1272
size_length = 16
for m in range(6):
    TCP_speed_target[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('TCP_speed_target',TCP_speed_target)

#Todo: test 8 inputs
#Current state of the digital inputs. NOTE: these are bits encoded as int64_t, e.g. a value of 5 corresponds to bit 0 and bit 2 set high
Digital_input_bits = np.zeros(1)
start_mark = 1368
size_length = 16
for m in range(1):
    TCP_speed_target[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Digital_input_bits',Digital_input_bits)

#Temperature of each joint in degrees celsius
Motor_temperatures = np.zeros(6)
start_mark = 1384
size_length = 16
for m in range(6):
    Motor_temperatures[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Motor_temperatures',Motor_temperatures)

#Controller realtime thread execution time
Controller_Timer = np.zeros(1)
start_mark = 1480
size_length = 16
for m in range(1):
    Controller_Timer[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Controller_Timer',Controller_Timer)

#A value used by Universal Robots software only
Test_value = np.zeros(1)
start_mark = 1496
size_length = 16
for m in range(1):
    Test_value[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Test_value',Test_value)

#Robot mode
Robot_Mode = np.zeros(1)
start_mark = 1512
size_length = 16
for m in range(1):
    Robot_Mode[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Robot_Mode',Robot_Mode)

#Joint control modes
Joint_Modes = np.zeros(6)
start_mark = 1528
size_length = 16
for m in range(6):
    Joint_Modes[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Joint_Modes',Joint_Modes)

#Safety Mode
Safety_Mode = np.zeros(1)
start_mark = 1624
size_length = 16
for m in range(1):
    Safety_Mode[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Safety_Mode',Safety_Mode)

#Used by Universal Robots software only
None_value = np.zeros(6)
start_mark = 1640
size_length = 16
for m in range(6):
    None_value[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('None_value',None_value)

#Tool x,y and z accelerometer values (software version 1.7)
Tool_Accelerometer_values = np.zeros(3)
start_mark = 1736
size_length = 16
for m in range(3):
    Tool_Accelerometer_values[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Tool_Accelerometer_values',Tool_Accelerometer_values)

#Used by Universal Robots software only
None_value = np.zeros(6)
start_mark = 1784
size_length = 16
for m in range(6):
    None_value[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('None_value',None_value)

#Speed scaling of the trajectory limiter
Speed_scaling = np.zeros(1)
start_mark = 1880
size_length = 16
for m in range(1):
    Speed_scaling[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Speed_scaling',Speed_scaling)

#Norm of Cartesian linear momentum
Linear_momentum_norm = np.zeros(1)
start_mark = 1896
size_length = 16
for m in range(1):
    Linear_momentum_norm[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Linear_momentum_norm',Linear_momentum_norm)

#Used by Universal Robots software only
None_value = np.zeros(1)
start_mark = 1912
size_length = 16
for m in range(1):
    None_value[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('None_value',None_value)

#Used by Universal Robots software only
None_value = np.zeros(1)
start_mark = 1928
size_length = 16
for m in range(1):
    None_value[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('None_value',None_value)

#Masterboard: Main voltage
V_main = np.zeros(1)
start_mark = 1944
size_length = 16
for m in range(1):
    V_main[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('V_main',V_main)

#Masterboard: Robot voltage (48V)
V_robot = np.zeros(1)
start_mark = 1960
size_length = 16
for m in range(1):
    V_robot[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('V_robot',V_robot)

#Masterboard: Robot current
I_robot = np.zeros(1)
start_mark = 1976
size_length = 16
for m in range(1):
    I_robot[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('I_robot',I_robot)

#Actual joint voltages
V_actual = np.zeros(6)
start_mark = 1992
size_length = 16
for m in range(6):
    V_actual[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('V_actual',V_actual)

#Digital outputs
Digital_outputs = np.zeros(1)
start_mark = 2088
size_length = 16
for m in range(1):
    Digital_outputs[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Digital_outputs',Digital_outputs)

#Program state
Program_state = np.zeros(1)
start_mark = 2104
size_length = 16
for m in range(1):
    Program_state[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Program_state',Program_state)

#Elbow position
Elbow_position = np.zeros(3)
start_mark = 2120
size_length = 16
for m in range(3):
    Elbow_position[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Elbow_position',Elbow_position)

#Elbow velocity
Elbow_velocity = np.zeros(3)
start_mark = 2168
size_length = 16
for m in range(3):
    Elbow_velocity[m] = struct.unpack('!d', ur_msg[start_mark+m*size_length:start_mark+(m+1)*size_length].decode('hex'))[0]
print('Elbow_velocity',Elbow_velocity)

s.close()
