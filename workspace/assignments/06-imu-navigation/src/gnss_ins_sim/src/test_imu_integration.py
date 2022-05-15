import os


from gnss_ins_sim.sim import imu_model
from gnss_ins_sim.sim import ins_sim
from imu_integration import IMUIntegration
import numpy as np
from poseinfo import RotationInfo

class TestImuIntegration(object):
    def __init__(self):

        return
    def get_gnss_ins_sim(self, motion_def_file, fs_imu, fs_gps):
        '''
        Generate simulated GNSS/IMU data using specified trajectory.
        '''
        #### choose a built-in IMU model, typical for IMU381
        imu_err = 'mid-accuracy'
        # generate GPS and magnetometer data
        imu = imu_model.IMU(accuracy=imu_err, axis=6, gps=False)
    
        # init simulation:
        sim = ins_sim.Sim(
            [fs_imu, fs_gps, fs_imu],
            motion_def_file,
            ref_frame=1,
            imu=imu,
            mode=None,
            env=None,
            algorithm=None
        )
        
        # run:
        sim.run(1)
    
        # get simulated data:
        print(
            "Simulated data size {}".format(
                len(sim.dmgr.get_data_all('gyro').data[0])
            )
        )
    
        # imu measurements:
        step_size = 1.0 / fs_imu
        for i, (gyro, accel, p, v, q) in enumerate(
            zip(
                # a. gyro
                sim.dmgr.get_data_all('ref_gyro').data, 
                # b. accel
                sim.dmgr.get_data_all('ref_accel').data,
                sim.dmgr.get_data_all('ref_pos').data,
                sim.dmgr.get_data_all('ref_vel').data,
                sim.dmgr.get_data_all('ref_att_quat').data
            )
        ):
            yield [i * step_size, gyro,accel, p, v, q]
            
            
    def run(self):
        # parse params:
        motion_def_name = 'demo.csv' #rospy.get_param('motion_file')
        sample_freq_imu = 100.0               #rospy.get_param('sample_frequency/imu')
        sample_freq_gps = 10.0                #rospy.get_param('sample_frequency/gps')
        
        
        # generate simulated data:
        motion_def_path = os.path.join(
            os.path.dirname(__file__), '../', 'config', 'motion_def', motion_def_name
        )
        imu_simulator = self.get_gnss_ins_sim(
            # motion def file:
            motion_def_path,
            # gyro-accel/gyro-accel-mag sample rate:
            sample_freq_imu,
            # GPS sample rate:
            sample_freq_gps
        )
    
        imu_int = IMUIntegration()
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
    obj= TestImuIntegration()
    obj.run()