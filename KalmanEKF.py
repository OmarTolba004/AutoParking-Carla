import numpy as np
import math
import matplotlib.pyplot as plt # Debugging

class EKF:
 
    def __init__(self, IMUNoise, GPSNoise, DT):
        '''
        Process Noise:Process noise represents the uncertainty or randomness
        in the evolution of the system state over time.
        It includes factors such as unmodeled dynamics, disturbances, or inaccuracies in the system model.
        In the Kalman filter, the process noise is typically assumed to be Gaussian (normally distributed) with mean zero.
        where Q:The covariance matrix which quantifies the statistical properties of the process noise. 
        '''
        self.Q = np.diag([
                    0.1,  # variance of location on x-axis
                    0.1,  # variance of location on y-axis
                    np.deg2rad(1.0),  # variance of yaw angle
                    1.0  # variance of velocity
                ]) ** 2  # predict state covariance
        '''
        Measurement noise is assumed to be Gaussian (normally distributed) with mean zero.
        It is typically denoted by 𝛿 and represents the difference between the true measurement and the measured value.
        The covariance matrix R quantifies the statistical properties of the measurement noise.
        Similar to the process noise covariance matrix Q, R is also a square matrix where each element represents the covariance between different measurement variables.
        '''
        self.R = np.diag([1.0, 1.0]) ** 2  # Observation x,y position covariance
        
        # Input and measurement noise
        '''
        Input noise is a diagonal matrix with first element
        in the diagonal represents the noise in longituidinal velocity and second element represents 
        the noise in the angular velocity
        '''
        self.IMUNoise = IMUNoise
        '''
        GPS noise is a diagonal matrix with first element
        in the diagonal represents the noise in x pos and second element represents 
        the noise in the y pos
        '''
        self.GPSNoise = GPSNoise
        
        # Tick time
        self.DT = DT

        # DEBUGGING
        print(self.Q, "\n", self.R, '\n', self.IMUNoise, '\n', self.GPSNoise)

    # Motion model funciton: refer to the "Sensor Fusion with EKF" pdf for details
    def motion_model(self,x, u):

        F = np.array([[1.0, 0, 0, 0],
                    [0, 1.0, 0, 0],
                    [0, 0, 1.0, 0],
                    [0, 0, 0, 0]])

        B = np.array([[self.DT * math.cos(x[2, 0]), 0],
                    [self.DT * math.sin(x[2, 0]), 0],
                    [0.0, self.DT],
                    [1.0, 0.0]])

        x = F @ x + B @ u
        return x
    
    # Observation model funciton: refer to the "Sensor Fusion with EKF" pdf for details
    def observation_model(self, x):
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        z = H @ x

        return z    
    
    # Jacobian of Motion model, refer to the "Sensor Fusion with EKF" pdf for details
    def jacob_f(self, x, u):
        yaw = x[2, 0]
        v = u[0, 0]
        jF = np.array([
            [1.0, 0.0, -self.DT * v * math.sin(yaw), self.DT * math.cos(yaw)],
            [0.0, 1.0, self.DT * v * math.cos(yaw), self.DT * math.sin(yaw)],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]])

        return jF

    # Jacobian of Observation Model, refer to the "Sensor Fusion with EKF" pdf for details
    def jacob_h(self):
        jH = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        return jH
    
    # EKF prediction and correction steps
    def ekf_estimation(self, xEst, PEst, z, u):
        #  Predict
        xPred = self.motion_model(xEst, u)
        jF = self.jacob_f(xEst, u)
        PPred = jF @ PEst @ jF.T + self.Q

        #  Correct
        jH = self.jacob_h()
        zPred = self.observation_model(xPred)
        y = z - zPred
        S = jH @ PPred @ jH.T + self.R
        K = PPred @ jH.T @ np.linalg.inv(S)
        xEst = xPred + K @ y
        PEst = (np.eye(len(xEst)) - K @ jH) @ PPred

        return xEst, PEst
    
    #DEBUGGING
    def calc_input(self):
        v = 1.0  # [m/s]
        yawrate = 0.1  # [rad/s]
        u = np.array([[v], [yawrate]])
        return u
    #DEBUGGING
    def observation(self, xTrue, xd, u):
        xTrue = self.motion_model(xTrue, u)

        # add noise to gps x-y
        z = self.observation_model(xTrue) + self.GPSNoise @ np.random.randn(2, 1)

        # add noise to input
        ud = u + self.IMUNoise @ np.random.randn(2, 1)

        xd = self.motion_model(xd, ud)

        return xTrue, z, xd, ud

    #DEBUGGING
    def mainalias(self):

        time = 0.0
        SIM_TIME = 50.0  # simulation time [s]
        show_animation = True

        # State Vector [x y yaw v]'
        xEst = np.zeros((4, 1))
        xTrue = np.zeros((4, 1))
        PEst = np.eye(4)

        xDR = np.zeros((4, 1))  # Dead reckoning

        # history
        hxEst = xEst
        hxTrue = xTrue
        hxDR = xTrue
        hz = np.zeros((2, 1))

        while SIM_TIME >= time:
            time += self.DT
            u = self.calc_input()

            xTrue, z, xDR, ud = self.observation(xTrue, xDR, u)

            xEst, PEst = self.ekf_estimation(xEst, PEst, z, ud)

            # store data history
            hxEst = np.hstack((hxEst, xEst))
            hxDR = np.hstack((hxDR, xDR))
            hxTrue = np.hstack((hxTrue, xTrue))
            hz = np.hstack((hz, z))

            if show_animation:
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                plt.plot(hz[0, :], hz[1, :], ".g")
                plt.plot(hxTrue[0, :].flatten(),
                        hxTrue[1, :].flatten(), "-b")
                plt.plot(hxDR[0, :].flatten(),
                        hxDR[1, :].flatten(), "-k")
                plt.plot(hxEst[0, :].flatten(),
                        hxEst[1, :].flatten(), "-r")
                plt.axis("equal")
                plt.grid(True)
                plt.pause(0.001)