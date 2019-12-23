import optoforce
import serial
import time
import sys
import os
import numpy as np
_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.append(_root_path)


f = optoforce.OptoforceDriver("/dev/ttyACM0", "s-ch/6-axis" ,[[10, 10, 10,1000,1000,1000]])

def detectCollision(min_fx = 20.0,min_fy = 20.0,min_fz = 20.0,min_tx = 5,min_ty = 5,min_tz = 5):
    f._serial.reset_input_buffer()
    FT = f.read().force
    print(FT)
    if FT[0][0] > min_fx or FT[0][1] > min_fy or FT[0][2] > min_fz or FT[0][3] > min_tx or FT[0][4] > min_ty or FT[0][5] > min_tz:
        return True
    else:
        return False


#example
# go to pont A and detect collision
if __name__ =='__main__':
    #
    # print(detectCollision())
    while True:
        f = optoforce.OptoforceDriver("/dev/ttyACM0", "s-ch/6-axis" ,[[10, 10, 10,1000,1000,1000]])
        time.sleep(1)
        FT = f.read().force
        print(FT)
