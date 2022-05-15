import numpy as np
import math
from poseinfo import ypr2R, RotationInfo

class ImuGen(object):
    def __init__(self):
        self.data = {}
    
            
        self.gyro_bias = np.zeros(3)
        self.acc_bias = np.zeros(3)
        self.seq = 0
        self.gw = np.array([0,0,-9.81007])
        return
    def get_data_all(self, data_name):
        if data_name in self.data:
            return self.data[data_name]
        else:
            return None
    def add_data(self, data_name, data):
        if not data_name in self.data:
            self.data[data_name] = []
        self.data[data_name].append(data)
        return
    def MotionModel_Curve(self, t):
        #constant acceleration straight line motion model
        #y forward, x, left, z, upward
        v = 5
        acc = 0.25
        K = 2 * math.pi/ 10
        K2 = K*K
        ellipse_x = 3
        vz = 0
        
        position = np.array([ellipse_x * math.sin( K * t), v * t, vz*t])
        dp = np.array([K * ellipse_x * math.cos(K*t), v,vz])
        ddp = np.array([-K2 * ellipse_x * math.sin(K*t),0,0])
        
        y = dp[0]
        x = dp[1]
        yaw = -math.atan2(y, x)
        
        K=0
      
        eulerAngles = np.array([0.025*K*t,0,yaw]) 
        
        x2plusy2 = x * x + y * y
        dfdy= x/ x2plusy2
        dfdx = -y/x2plusy2
        
        dydt = ddp[0]
        dxdt = ddp[1]
        
        yaw_rate = -(dfdy*dydt + dfdx * dxdt)
        eulerAnglesRates = np.array([0.25*K,0,yaw_rate]) 
        
        
        Rwb = ypr2R(eulerAngles[::-1] * 180/math.pi)
        r, p, y = eulerAngles
        sr = math.sin(r)
        tp = math.tan(p)
        cr = math.cos(r)
        cp = math.cos(p)
        Reg = np.array([[1, sr*tp, cr*tp],[0, cr, -sr], [0, sr/cp, cr/cp]])
        Rge = np.transpose(Reg)
        
        imu_gyro = Rge.dot(eulerAnglesRates)
        
        
        imu_acc = Rwb.transpose().dot(( ddp -  self.gw ))
        
        self.add_data('ref_pos', position)
        self.add_data('ref_vel', dp)
        self.add_data('ref_att', eulerAngles)
        q = RotationInfo().construct_fromR(Rwb[:3,:3]).q
        x,y,z,w = q 
        q = np.array([w,x,y,z])
        self.add_data('ref_att_quat', q)
        
        self.add_data('ref_gyro', imu_gyro)
        self.add_data('ref_acc', imu_acc)
        self.add_data('ts', t)

        self.seq += 1
        return
    def gen_data(self):
        t = 0
        t_end = 10
        imu_freq = 100
        while t< t_end:
            self.MotionModel_Curve(t) 
            t += 1.0/imu_freq
        return
    def run(self):
        return
    
    
if __name__ == "__main__":   
    obj= ImuGen()
    obj.run()