"""************************************************************************************************************************
*   File Name: Controller.py
*   Authors: Omar Tolba
*   Last modifing Date: 29/4/2024
***************************************************************************************************************************
*   Description: longitduinal and lateral control implementation
************************************************************************************************************************"""
import numpy as np
import sys
import pidCPython as pid #importing pid Python wrapper 

sys.path.append('/home/omar/gp/va/treasure of Algos/PythonRobotics/')
from utils.angle import angle_mod


class LateralControl:
    def __init__(self, L= 2.9, max_steer = np.radians(70.0), k = 0.5):
        self.L = L  # [m] Wheel base of vehicle
        self.MaxSteer = max_steer # [rad] max steering angle
        self.K = k  # control gain
    

    def stanley_control(self, state, cx, cy, cyaw, last_target_idx):
        """
        Stanley steering control.

        :param state: (State object)
        :param cx: ([float])
        :param cy: ([float])
        :param cyaw: ([float])
        :param last_target_idx: (int)
        :return: (float, int)
        """
        current_target_idx, error_front_axle = self.calc_target_index(state, cx, cy)

        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # theta_e corrects the heading error
        theta_e = self.normalize_angle(cyaw[current_target_idx] - state[2][0])
        # theta_d corrects the cross track error
        theta_d = np.arctan2(self.K * error_front_axle, state[3][0])
        # Steering control
        delta = theta_e + theta_d

        return delta, current_target_idx
    
    def calc_target_index(self, state, cx, cy):
        """
        This function determines the target point on the trajectory that the vehicle should aim for based on its current state. This target point
        is used to calculate the cross-track error which is essential for adjusting the vehicle's steering. The index of this target point and the error are then used by the Stanley control
        algorithm to calculate the appropriate steering angle to minimize this error and follow the trajectory accurately

        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = state[0][0] + self.L * np.cos(state[2][0])
        fy = state[1][0] + self.L * np.sin(state[2][0])

        # Search nearest point index
        # Calculating the distance between each point in x and y and the front wheel and put them in the lists dx and dy repectively
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        # Calculating the Eculedian distance
        d = np.hypot(dx, dy)
        # The index of the trajectory point that is closest to the vehicle's front axle, extracting the minumum index of the array of distances
        target_idx = np.argmin(d)

    

        # Project RMS error onto front axle vector, front_axle_vec is a unit vector perpendicular to the vehicle's heading.
        front_axle_vec = [-np.cos(state[2][0] + np.pi / 2),
                        -np.sin(state[2][0] + np.pi / 2)]
        # The cross-track error, which is the perpendicular distance from the front axle of the vehicle to the target trajectory.
        # error_front_axle is the dot product of the displacement vector from the front axle to the nearest trajectory point and the front axle's perpendicular vector.
        # This gives the signed distance, indicating whether the vehicle is to the left or right of the trajectory.
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle
    

    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].

        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        return angle_mod(angle)


class LongitudinalController: 
    def __init__(self, l_kp = 2.0, l_ki= 0.001, l_kd= 0.0,l_T= 0.01, l_maxLimit= 10.0, l_minLimit= -10.0, l_maxLimitInt= 5.0, l_minLimitInt= -5.0, l_tau= 0.02):
        self.pid = pid.pid(l_kp , l_ki, l_kd,l_T, l_maxLimit, l_minLimit, l_maxLimitInt, l_minLimitInt, l_tau)
    
    def update(self, setpoint, measurement) -> float:
        return self.pid.update(setpoint, measurement)