import numpy as np
import KalmanEKF


class SensorFusion:

    def __init__(self, IMUNoise, GPSNoise, DT):
        # Using EKF filter
        self.filter = KalmanEKF.EKF(IMUNoise, GPSNoise, DT)

    def observation(self):
        pass


    def estimate(self, xEst, PEst, z, u):
        xEst_new, pEst_new = self.filter.ekf_estimation(xEst, PEst, z, u)
        return xEst_new, pEst_new