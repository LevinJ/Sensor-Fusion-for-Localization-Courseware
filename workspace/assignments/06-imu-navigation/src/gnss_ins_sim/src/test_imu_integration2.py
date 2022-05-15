import os


from imu_gen import ImuGen
from imu_integration import IMUIntegration
import numpy as np
from poseinfo import RotationInfo

class TestImuIntegration2(object):
    def __init__(self):

        return
    def get_sim_data(self):
        imu_gen = ImuGen()
        imu_gen.gen_data()
        
        for item in zip(
                imu_gen.get_data_all('ts'), 
                imu_gen.get_data_all('ref_gyro'), 
                # b. accel
                imu_gen.get_data_all('ref_acc'),
                imu_gen.get_data_all('ref_pos'),
                imu_gen.get_data_all('ref_vel'),
                imu_gen.get_data_all('ref_att_quat')
            ):
            yield item
        
            
    def run(self):
        # parse params:
        
        imu_simulator = self.get_sim_data()
        imu_int = IMUIntegration()
        imu_int.gw = ImuGen().gw
        try:
            while True:
                t, w, a, p, v, q = next(imu_simulator)
                if not imu_int.is_init:
                    imu_int.set_ini_pva(p, v, q)
                imu_int.process(t, w, a)
                
                print("err = {},p = {}, gt_p = {}".format(np.linalg.norm(p - imu_int.p), imu_int.p, p))
                
        except StopIteration:
            pass
        
        
    
                    
        return
    
    
if __name__ == "__main__":   
    obj= TestImuIntegration2()
    obj.run()