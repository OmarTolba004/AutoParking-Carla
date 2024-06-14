"""************************************************************************************************************************
*   File Name: Car.py
*   Authors: Omar Tolba
*   Last modifing Date: 14/6/2024
***************************************************************************************************************************
*   Description: Car Class implementation 
************************************************************************************************************************"""
import carla
import time
import random 
import numpy as np
import cv2
import math #for sqrt function
import pidCPython as pid #importing pid Python wrapper 
import matplotlib.pyplot as plt # for debugging
import KalmanEKF as SF
import sys
import Controller 


# the following for path tracking
##########
sys.path.append('/home/omar/gp/carla/download/CARLA_Latest/PythonAPI/carla') # for calculating the route using carla Utils
from agents.navigation.global_route_planner import GlobalRoutePlanner # for calculating the route using carla Utils
###########


'''
Todos:
    1- [Todo] : Control car lights when applying specifing steering angle
    2- [todo] : make front camera and rear camera 
    3- [todo] : add ultrasonics 
    4- [todo] : agree upon adel alogrithms api prototypes
    4- [todo] : add lidar for simulation only [postponed]
'''
class Car:

    # ----------------
    ## Class Variables
    # ----------------

    L = 2.9  # [m] Wheel base of vehicle
    maxSteer = np.radians(70.0) # max steering angle in radian
    # Camera offset on the Z and X direction
    CAMERA_POS_Z = 1.5
    CAMERA_POS_X = 0.4
    # Sensors Data Dictionary 
    # IMU velocity 
    IMUVelocity = 0
    IMUVelocityX = 0
    
    GPS_NOISE = np.diag([0.5, 0.5]) ** 2 
    INPUT_NOISE = np.diag([1.0, np.deg2rad(30.0)]) ** 2

    # This counter is used to count how many tick have been executed (used to ignore sensors reading at the begining )
    execCounter = 0
    # Current speed from simulation
    currentSpeed = 0

    # ----------------
    ## Debugging Class Variables
    # ----------------

    ## Different openCv display parameters
    # Picking specific font
    OpenCVFont = cv2.FONT_HERSHEY_SIMPLEX
    # org - defining lines to display telemetry values on the screen
    OpenCVOrg = (30, 30) # this line will be used to show current speed
    OpenCVOrg2 = (30, 50) # this line will be used for future steering angle
    OpenCVOrg3 = (30, 70) # and another line for future telemetry outputs
    OpenCVOrg4 = (30, 90) # and another line for future telemetry outputs
    OpenCVOrg3 = (30, 110) # and another line for future telemetry outputs
    OpenCVFontScale = 0.5
    # white color
    OpenCVColor = (255, 255, 255)
    # Line thickness of 2 px
    OpenCVThickness = 1

    # ----------------
    ## Class Constructor
    # ----------------
    def __init__(self, client, world, DT):

        # ----------------
        ## Setting up the world and the vehicle
        # ----------------
        # Setting the client and the world instance variables
        self.client = client
        self.world = world
        # apply world settings
        self.DT = DT
        self.__apply_carla_settings(DT)
        # Spawning the Car 
        self.__spawn_vehicle()
        # required initialization
        self.__initialize_controllers()
        self.__initialize_sensors()
        self.__initialize_sensor_fusion()
   
    def __apply_carla_settings(self, DT):
        ''' 
        This function apply the following carla simulation settings : 
        1 - Activating synchrounous mode 
        2 - use fixed time step 
        3- applies phyiscs substepping
         Synchronous mode + fixed time-step. The client will rule the simulation. The time step will be fixed.
         The server will not compute the following step until the client sends a tick.
         This is the best mode when synchrony and precision is relevant.
         Especially when dealing with slow clients or different elements retrieving information.
         refer to https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/#setting-synchronous-mode fro more documentation
        4- adjusting the weather
        '''
        # Get world settings
        settings = self.world.get_settings()
        #Setting the weather
        weather = carla.WeatherParameters()
        weather.sun_altitude_angle = -90
        # change settings
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = DT
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10
        # Apply
        self.world.set_weather(weather)
        self.world.apply_settings(settings)

    def __spawn_vehicle(self):
        # Get all avialable spawn points
        self.spawnPoints = self.world.get_map().get_spawn_points()
        # Getting start point of the Car
        self.startPoint = self.spawnPoints[0]
        # Getting specific Car blueprint
        self.carBluePrint = self.world.get_blueprint_library().find('vehicle.lincoln.mkz_2020')
        # Setting the spawned Car as the ego vehicle and specifying the color to black
        self.carBluePrint.set_attribute('role_name', 'hero')
        self.carBluePrint.set_attribute('color', '0 ,0 ,0')
        # Trying to spawn the car in the start point
        self.car = self.world.try_spawn_actor(self.carBluePrint, self.startPoint) 
        # Checking if car spawned correctly 
        if(self.car is None) :
            print("Couldn't spawn the lincoln mkz 2020 car")
            raise RuntimeError("Failed to spawn vehicle")

    #------------
    ## Initializations
    #-------------
    def __initialize_controllers(self):
        # initialize the controller [todo : make the controller class for couples lateral and longituduinal control]
        self.lateralController = Controller.LateralControl(L=self.L, max_steer= self.maxSteer, k=0.5, calculate_over_span=True, index_searching_span_size=5)

        # Defining Throttle PID, [todo]: need to encapsulated within private function
        self.longitudinalController = Controller.LongitudinalController(l_kp= 65, l_ki=0.06, l_kd=0.5,l_maxLimit= 100, l_minLimit= 0) 
       
    def __initialize_sensors(self):
        # Configuring Camera and its callback
        self.camera = self.__configure_camera_sensor()
        # Setting Up IMU Sensor and its callback
        self.IMU = self.__configure_imu_sensor()
        # Setting UP GNSS Sensor and its callback
        self.GNSS = self.__configure_gnss_sensor()

    def __initialize_sensor_fusion(self):
        # Creating instance out of EKF filter
        self.sf = SF.EKF(np.zeros((2,1)), np.zeros((2,1)), self.DT)

        # initialize True position 
        self.xTrue = np.array([[self.startPoint.location.x],
                               [self.startPoint.location.y],
                               [0],
                               [self.car.get_velocity().x]]) 
        # initialize estimate position (the value corrected by EKF in correction step)
        self.xEst  = np.copy(self.xTrue)
        # Dead reckoning init value (the value calculated from the motion model and the action)
        self.xDR = np.copy(self.xTrue)
        # Variance init value
        self.PEst = np.eye(4)
        
        # Action parameters, velcoity and yawrate(angular velocity)
        self.u = np.zeros((2,1))
        # Measurement init values, x-pos and y-pos
        self.z = np.zeros((2,1))

        ##Debugging 
        self.hxDR = self.xTrue
        self.hxTrue = self.xTrue
        self.hxEst = self.xEst

    def __sensorFusionObservation(self):
        '''
        1 - update xTrue
        2- get input (already done)
        3- get measurement (already done)
        4- calculate DR using motion motion model
        '''
        # Update xTrue with car current pos
        self.xTrue = np.array([[self.car.get_transform().location.x],
                               [self.car.get_transform().location.y],
                               [0],
                               [self.car.get_velocity().x]])
        
        
        # Calculate Dead reckoning based on action model
        self.xDR= self.sf.motion_model(self.xDR, self.u)


        # UnComment for debugging
        self.hxDR = np.hstack((self.hxDR, self.xDR))
        self.hxTrue = np.hstack((self.hxTrue, self.xTrue))
               
    def apply_sensor_fusion(self):
        # apply obesrvation
        self.__sensorFusionObservation()
        # estimating new position 
        self.xEst, self.PEst = self.sf.ekf_estimation(self.xEst, self.PEst, self.z, self.u)
        
        self.hxEst = np.hstack((self.hxEst, self.xEst))
        
        ## Uncomment for sensor fusion debugging
        # plt.cla()
        #     # for stopping simulation with the esc key.
        # plt.gcf().canvas.mpl_connect('key_release_event',
        #         lambda event: [exit(0) if event.key == 'escape' else None])
        # plt.plot(self.hxEst[0, :].flatten(),
        #              self.hxEst[1, :].flatten(), "-r", label ="ekf estimation")
        # plt.plot(self.hxDR[0, :].flatten(),
        #              self.hxDR[1, :].flatten(), "-k", label="dead reckoning")
        # plt.plot(self.hxTrue[0, :].flatten(),
        #              self.hxTrue[1, :].flatten() , "-b", label="ture position")
        # plt.axis("equal")
        # plt.legend()
        # plt.grid(True)
        # plt.pause(0.001)

    #Funciton to generate waypoints and get their rotation angles
    def __get_route_info(self, startPoint, endPoint, samplingResoloution = 2):
        x = list()
        y =list()
        yaw = list()
        grp = GlobalRoutePlanner(self.world.get_map(), samplingResoloution)
        # calculate the route
        route = grp.trace_route(startPoint, endPoint)
        # loop over waypoints
        for waypoint in route:
            x.append(waypoint[0].transform.location.x)
            y.append(waypoint[0].transform.location.y)
            yaw.append(np.deg2rad(waypoint[0].transform.rotation.yaw))
            self.world.debug.draw_string(location = waypoint[0].transform.location, text = '^',life_time = 70) # for debugging
        return x,y,yaw

    def initialize_path_tracking(self, startPoint, endPoint):
        '''
        This Function should be invoked before calling self.applyPathTracking() if new start and end point needed
        '''
        '''
        get cx : array of waypoints x pos
        get cy : array of waypoints y pos
        get yaw : array of waypoints car orientation angle
        '''
        self.cx, self.cy, self.cyaw = self.__get_route_info(startPoint, endPoint)
        # get last index of the waypoints
        self.lastIndex = len(self.cx) - 1
        # calculate Target index 
        self.targetIndex, _ = self.lateralController.calc_target_index(self.xEst, self.cx, self.cy)

    def apply_path_tracking(self):
        # apply the stanley control
        steeringAngle, self.targetIndex = self.lateralController.stanley_control(self.xEst, self.cx, self.cy, self.cyaw, self.targetIndex)
        # Calculate steering openning from [-1 , 1]
        steerOpenning = steeringAngle / self.maxSteer
        # Setting car velocity and angular velocity
        self.set_vehicle_state(40, steerOpenning)        

    def __configure_camera_sensor(self):
        # Getting Camera BluePrint
        cameraBluePrint = self.world.get_blueprint_library().find('sensor.camera.rgb')
        # Camera mounting point relative to the car
        cameraMountingPoint = carla.Transform(carla.Location(z = self.CAMERA_POS_Z, x=self.CAMERA_POS_X))
        # Spawning the Camera
        camera = self.world.try_spawn_actor(cameraBluePrint, cameraMountingPoint, attach_to=self.car)
        # Checking if Camera spawned correctly 
        if(camera is None):
            print("Couldn't spawn the Camera sensor")
            raise RuntimeError("Failed to spawn camera")
        
        # Getting camera image width and height to initalize dict key value-pair with
        self.cameraImageWidth = cameraBluePrint.get_attribute('image_size_x').as_int()
        self.cameraImageHeight = cameraBluePrint.get_attribute('image_size_y').as_int()
        # initialize camera data with those height and width 
        # np.zeros is numpy utility that Return a new array of given shape and type, filled with zeros.
        # we want array of rows equal to height, columns equal to width and we have 4 channels (red, green, blue and alpha)
        self.cameraDataDict = {'image' : np.zeros((self.cameraImageHeight, self.cameraImageWidth, 4))}
        # Camera listen call back
        camera.listen(lambda image: self.__camera_callback(image, self.cameraDataDict))

        return camera
    # Protected class method, Camera callback
    def __camera_callback(self, image, dataDict):
        '''
        1- Inserting the data into the pass by ref dataDict parameter
        2- numpy reshape is a utility that Gives a new shape to an array without changing its data.
        3- The new shape should be compatible with the original shape. If an integer, then the result will be a 1-D array of that length. 
        4- numpy copy is a utility that Return an array copy of the given object.
        5-image raw data is a Flattened array of pixel data, use reshape to create an image array, where A flattened array of pixel data refers to a one-dimensional array 
        that contains the pixel values of an image in a format where all the pixel values are placed sequentially in memory,
        row by row or column by column, depending on the image's storage convention.
        6- so what we are doing is converting the image to flattened array then reshape it to the needed shape which is 
        array of rows equal to height, columns equal to width and we have 4 channels (red, green, blue and alpha)
        '''
        dataDict['image'] = np.reshape(np.copy(image.raw_data), (self.cameraImageHeight, self.cameraImageWidth, 4))

    def __configure_imu_sensor(self):
        # Getting blue print
        IMUBluePrint = self.world.get_blueprint_library().find('sensor.other.imu')
        #[todo] : set blue print attr (such as std deviations and noise seeds)
        # getting IMU mouting point
        IMUMountingPoint = carla.Transform(carla.Location(z = 0, x=0))
        # Spawning the IMU and store it in the self pointer
        IMU = self.world.try_spawn_actor(IMUBluePrint, IMUMountingPoint, attach_to=self.car)
        # Checking if IMU spawned correctly
        if(IMU is None):
            print("Failed to spawn IMU")
            raise RuntimeError("Failed to spawn IMU")
        # Configure listenning function
        IMU.listen(lambda data : self.__imu_callback(data))

        return IMU

    def __imu_callback(self, data):
        if(self.execCounter < 10):
            # This loop to ignore first reading which are garbage values due to car spawning from air
            return

        '''
        All the sensors use the UE coordinate system (x-forward, y-right, z-up),
        and return coordinates in local space. 
        so to get angular velocity around longitudinal axis, we use the gyroscope reading in the x-driection
        and same with longitudinal axis
        '''
        # Calculating velocity based upon accelration reading in the longitudinal axis
        self.IMUVelocityX = self.__calculate_velocity(data.accelerometer.x, self.IMUVelocityX, self.DT)
        self.IMUVelocity = math.sqrt(self.IMUVelocityX**2)
        self.currentSpeed = math.sqrt(self.car.get_velocity().x**2 +self.car.get_velocity().y**2)
        # Storing velocity in m/s and yawrate in rad/s
        self.u = np.array([[self.IMUVelocity],[data.gyroscope.z]])
        self.u = self.u + self.INPUT_NOISE @ np.random.randn(2, 1)
    
    def __calculate_velocity(self, acceleration, initialVelocity, DT):
        """
        Calculate velocity from acceleration using numerical integration (Trapezoidal Rule).

        Parameters:
            acceleration (float): Acceleration value.
            initialVelocity (float, optional): Initial velocity. Default is 0.

        Returns:
            float: Velocity calculated from acceleration.
        """
        velocityChange = acceleration * DT
        velocity = velocityChange + initialVelocity
        return velocity
   
    def __configure_gnss_sensor(self):
        # Getting blue print
        GNSSBluePrint = self.world.get_blueprint_library().find('sensor.other.gnss')
        #[todo] : set blue print attr (such as std deviations and noise seeds)
        # getting GNSS mouting point
        GNSSMountingPoint = carla.Transform(carla.Location(z = 0, x=0))
        # Spawning the GNSS and store it in the self pointer
        GNSS = self.world.try_spawn_actor(GNSSBluePrint, GNSSMountingPoint, attach_to=self.car)
        # Configure listenning function
        GNSS.listen(lambda data : self.__gnss_callback(data))
        return GNSS

    def __gnss_callback(self, data):
        if(self.execCounter < 10):
            # This loop to ignore first reading which are garbage values due to car spawning from air
            return
        
        # Storing the measurement 
        self.z = np.array([[data.longitude],[data.latitude]])
        # for now I am using trasfrom values from the simulation as reading [todo]: see how to fix this
        self.z = np.array([[self.car.get_transform().location.x],[self.car.get_transform().location.y]])
        self.z = self.z + self.GPS_NOISE @ np.random.randn(2, 1) 

    # Method to open the front Camera OpenCvWindow
    def OpenCvFrontCameraInit(self):
        # Opening window named RGB Front Camera
        cv2.namedWindow('RGB Front Camera',cv2.WINDOW_AUTOSIZE)

    # Updatint openCv window view, need to be called periodically
    def OpenCvFrontCameraUpdate(self):
        image = self.cameraDataDict['image']
        # Adding the speed text to the winodow showing car camera
        image = cv2.putText(image, 'Speed: '+str(int(3.6 * self.currentSpeed))+' kmh', self.OpenCVOrg2, self.OpenCVFont, self.OpenCVFontScale, self.OpenCVColor, self.OpenCVThickness, cv2.LINE_AA)
        # Showing data over the window
        cv2.imshow('RGB Front Camera',self.cameraDataDict['image'])
        # Wait for a small amount of time (1 millisecond) and check if a key is pressed
        # If no key is pressed, continue; otherwise, exit the loop
        cv2.waitKey(1)
        #[todo] : add parameters to openCv window

    #------------
    ## State Utilitites
    #-------------
    def set_vehicle_state(self, desiredSpeed, steerOpenning):
        '''
            This is an initial function to control car state
            [todo] : do you need to pass sensor to the camera or use instance variables
        '''
        # Read velocity (which is vector in 3d) from the game in m/s
        # converting velocity to speed, then converting m/s to km/h
        imu_velocity_kmh = 3.6 * self.IMUVelocity
        # Controlling the car throttle based on the current speed and desired speed 
        self.car.apply_control(carla.VehicleControl(throttle=self.__adjust_throttle(desiredSpeed, imu_velocity_kmh), steer=steerOpenning)) 

    # Private method for Maintaining the speed using Controller
    def __adjust_throttle(self, desiredSpeed, currentSpeed):
        ''' 
        this is a very simple function to maintan desired speed
        s arg is actual current speed
        '''
        # Calculating Throttle openning percentage 
        throttleOpeningPercentage = self.longitudinalController.update(setpoint=desiredSpeed, measurement=currentSpeed)
        return throttleOpeningPercentage /100.0

              
    # Class destructor
    def __del__(self):
        # Destroying vehicle and sensors
        for actor in self.world.get_actors().filter('*vehicle*'):
            actor.destroy()
        for sensor in self.world.get_actors().filter('*sensor*'):
            sensor.destroy()
            sensor.stop()
        # Closing all openCv2 windows
        cv2.destroyAllWindows() #[todo] : close specific opened windows

