import scipy.optimize
import numpy as np
from numpy import cos, sin
import time
from pyrep.errors import ConfigurationError, ConfigurationPathError, IKError
from scipy.spatial.transform import Rotation

def get_rotation_part(H):
    return H[0:3,0:3]

def get_transition_part(H):
    return H[0:3,3]

def set_rotation_part(H, r:Rotation):
    R = r.as_matrix()
    H[0:3,0:3] = R
    return H

def set_position_part(H, position):
    H[0:3,3] = np.array(position.T)
    return H
    
class FrankaKinematics():
    '''
    provide Fk and IK function of franka panda
    '''
    def __init__(self):
        '''
        init some constants
        '''
        self.DH_parameter_list = {
            'j1':       {'a':0,         'd':0.333,  'alp':0},
            'j2':       {'a':0,         'd':0,      'alp':-np.pi/2},
            'j3':       {'a':0,         'd':0.316,  'alp':np.pi/2},
            'j4':       {'a':0.0825,    'd':0,      'alp':np.pi/2},
            'j5':       {'a':-0.0825,   'd':0.384,  'alp':-np.pi/2},
            'j6':       {'a':0,         'd':0,      'alp':np.pi/2},
            'j7':       {'a':0.088,     'd':0,      'alp':np.pi/2},
            'flange':   {'a':0,         'd':0.107,  'alp':0}, # theta = 0
            'gripper':  {'a':0,         'd':0.1034, 'alp':0}  # theta = 0
        }
        self.home_joints = [0, -np.pi/4, 0, -3 * np.pi/4, 0, np.pi/2, np.pi/4]
        
        self.joint_bonds = ((-2.8973,2.8973),(-1.7628,1.7628),(-2.8973,2.8973),
                            (-3.071,-0.0698),(-2.8973,2.8973),(-0.0175,3.7525),(-2.8973,2.8973))
        '''
        self.joint_bonds = ((2.8973, 1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973),
                            (-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973))
        '''
    def dh_home_matrix(self,theta,joint_name):

        '''
        Hx_x+1 = Rot_X_alp * Trans_X_a * Rot_Z_theta * Trans_Z_d
        
        c_theta         -s_theta        0           a

        s_theta*c_alp   c_theta*c_alp   -s_alp      -d*s_alp

        s_theta*s_alp   c_theta*s_alp   c_alp       d*c_alp

        0               0               0           1

        '''
        DH_parameter = self.DH_parameter_list[joint_name]
        a = DH_parameter['a']
        d = DH_parameter['d']
        alp = DH_parameter['alp']
        c_theta = cos(theta)
        s_theta = sin(theta)
        c_alp = cos(alp)
        s_alp = sin(alp)
        return np.array([
            [c_theta         ,-s_theta        ,0           ,a],
            [s_theta*c_alp   ,c_theta*c_alp   ,-s_alp      ,-d*s_alp],
            [s_theta*s_alp   ,c_theta*s_alp   ,c_alp       ,d*c_alp],
            [0               ,0               ,0           ,1]
        ])
    
    def fk(self, q):
        '''
        forward kenimatics of panda

        compute Homegenous Matrix form joint angle

        H = H0_1 * H1_2 * H2_3 * H3_4 * H4_5 * H5_6 * H6_7 * H_flange * H_gripper
        
        para
        ---
            q: joint angle(rad)

        speed
        ---
            ~= 0.1ms
        '''
        
        # check q
        q = np.array(q)
        '''
        if q.shape[0] != 7:
            raise ValueError("q's lenght should be (7)")
        '''
        # H = H0_1 * H1_2 * H2_3 * H3_4 * H4_5 * H5_6 * H6_7
        H = np.eye(4)
        for i, theta in enumerate(q):
            H = np.dot(H, self.dh_home_matrix(theta, 'j'+str(i+1)))

        # H = H * H_flange * H_gripper
        H = np.dot(H, self.dh_home_matrix(0,'flange'))
        return np.dot(H, self.dh_home_matrix(0, 'gripper'))
        #return H

    def ik(self, H_target, q_guess):
        '''
        Ik slover for panda
        para
        ---
            H_target: target Homegenous Matrix

            q_guess: init joint 
        
        return
        ---
            list[7]

        speed
        ---
            ~= 80ms
        '''
        def opt_fun(q):
            return np.linalg.norm(self.fk(q) - H_target)
        res = scipy.optimize.minimize(opt_fun,q_guess,
                                    method='L-BFGS-B', 
                                    bounds=self.joint_bonds, 
                                    tol=1e-8,
                                    options={'maxiter':1000})
        if res.success:
            return res.x
        else:
            raise IKError('d')
if __name__ == "__main__":
    franka = FrankaKinematics()
    start = time.time()
    for _ in range(100):
        a=franka.fk(franka.home_joints)
    end = time.time()
    print((end-start)/100)
    print(a)
    H = np.array([
        [1,0,0,0.3],
        [0,-1,0,0],
        [0,0,-1,0.5],
        [0,0,0,1]
    ])
    start = time.time()
    for _ in range(100):
        res = franka.ik(H,franka.home_joints)
    end = time.time()
    print((end-start)/100)
    #print(res)
    print(franka.fk([0,0,0,0,0,0,0]))
