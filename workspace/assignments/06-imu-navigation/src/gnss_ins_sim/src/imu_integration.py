from pyquaternion import Quaternion
import math
import numpy as np
from math import cos
from math import sin

class IMUIntegration(object):
    def __init__(self):
        self.p = None 
        self.v = None 
        self.q = None
        
        self.last_a = None
        self.last_w = None
        self.last_t = None
        
        self.gw = None
        return
    def set_ini_pva(self, p, v, q):
        self.p = p 
        self.v = v 
        w, x,y,z = q 
        self.q = Quaternion(w, x, y, z)
        return
    @property
    def is_init(self):
        return self.last_a is not None
    def process(self, t, w, a):
        if not self.is_init:
            self.last_t, self.last_w, self.last_a = t, w, a 
            return
        dt = t - self.last_t
        int_w = (self.last_w + w )/2.0 
        dtheta = int_w * dt
        dtheta_norm = np.linalg.norm(dtheta)
        if dtheta_norm < 1e-8:
            dq = Quaternion(w=1, x=0, y=0, z=0)
        else:
            dq_w = cos(dtheta_norm/2.0)
            dq_x, dq_y, dq_z = dtheta/dtheta_norm * sin(dtheta_norm/2.0)
            dq = Quaternion(w=dq_w, x=dq_x, y=dq_y, z=dq_z)
            dq = dq.normalised 
        
        
        last_R = self.q.rotation_matrix
        self.q = self.q * dq
        R = self.q.rotation_matrix
        int_a = 0.5 * (last_R.dot(self.last_a) + R.dot(a)) + self.gw
        print("int_a={}".format(int_a))
        
        last_v = self.v
        self.p = self.p + last_v * dt + 0.5 * int_a * dt* dt 
        self.v = last_v + int_a * dt
        
        #save last input
        self.last_t, self.w, self.last_a = t, w, a
            
        return