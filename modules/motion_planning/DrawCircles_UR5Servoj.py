import socket
import time
import numpy as np

robot_ip = "192.168.1.27" # The remote robot_ip
port = 30003
print("Starting Program of drawing 4 circles using servoj!")

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(10)
s.connect((robot_ip, port))

# set time step of servoj
dt = 0.01
# set the scale of wait time for execution of each waypoint, the small this number is, the fast the move is
# the wait time should be less the dt, otherwise the movement will stop at every waypoints
scale = 0.4

for j in range(4):
    # set radius of the jth circle
    R = 0.08 - 0.01*j
    t1 = time.time()
    for i in range(1100):
        if i>1000:
            theta = 3.1415/500 * 1000
        else:
            theta = 3.1415/500 * i
        x = -0.0253 + R*np.cos(theta)
        y = -0.53275418 + R*np.sin(theta)
        tcp_command = "servoj(get_inverse_kin(p [%s, %s,  0.14670, 3.14030481,  0.06973561, 0.0102991]), t=%s,gain=100)\n"%(x,y, dt)
        l = s.send(str.encode(tcp_command))
        time.sleep(dt*scale)
    t2=time.time()
    print("Time cost of drawing %sth circle: %s"%(j, t2-t1))


s.close()
