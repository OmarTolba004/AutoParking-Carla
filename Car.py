import carla
import time
import random 
import numpy as np
import cv2
import math #for sqrt function
import pidCPython as pid #importing pid Python wrapper 
import matplotlib.pyplot as plt # for debugging


'''
    modules needed:
        1- openCv
        2- numpy
'''

'''
Todos:
    1- [Todo] : control map layers, load and unload map layer using world object
    2- [Todo] : Control car lights when applying specifing steering angle
    3- [Todo] : work in synchronous mode
    4- [Todo] : customizing the weather
    5- [Todo] : destroying all sensors and cv2 cleanup
    6- [todo] : load specific town at the begining 
    7- [todo] : you may nned to add sleep time for camera to spawn
    8- [todo] : make front camera and rear camera
    9- [todo] : add ultrasonics and Imu sensors and ensure their readings are meaningful
    10- [todo] : agree upon adel alogrithms api prototypes
    11- [todo] : add lidar for simulation only
    12- [todo] : add GPS for simulation only 
    13- [todo] : implement funcitons to encapsulate setting car sensros and Controllers such as pid

'''
class Car:

    # ----------------
    ## Class Variables
    # ----------------
    # Camera offset on the Z and X direction
    CAMERA_POS_Z = 1.5
    CAMERA_POS_X = 0.4
    # Sensors Data Dictionary 
    # dataDict = {} [Todo]

    # ----------------
    ## Debugging Class Variables
    # ----------------
    # Initialize lists to store time and variable data for plotting
    speed_data = []

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
        self.__applyCarlaSettings(DT)
        # Defining Throttle PID, [todo]: need to encapsulated within private function
        self.throttlePID = pid.pid(l_kp= 65, l_ki=0.06, l_kd=0.5,l_maxLimit= 100, l_minLimit= 0) 
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
            exit()
        # ----------------
        ## Init values for some instance variables
        # ----------------
        self.currentSpeed = 0

        # ----------------
        ## Setting Sensors [todo]: need to encapsulated within private function
        # ----------------
        # Setting up IMU
        self.__configureIMUSensor()
        # Setting up the camera
        # Getting Camera BluePrint
        self.cameraBluePrint = self.world.get_blueprint_library().find('sensor.camera.rgb')
        # Camera mounting point relative to the car
        self.cameraMountingPoint = carla.Transform(carla.Location(z = self.CAMERA_POS_Z, x=self.CAMERA_POS_X))
        # Spawning the Camera
        self.camera = self.world.try_spawn_actor(self.cameraBluePrint, self.cameraMountingPoint, attach_to=self.car)
        # Checking if Camera spawned correctly 
        if(self.camera is None):
            print("Couldn't spawn the Camera sensor")
            exit()
        
        # Getting camera image width and height to initalize dict key value-pair with
        self.cameraImageWidth = self.cameraBluePrint.get_attribute('image_size_x').as_int()
        self.cameraImageHeight = self.cameraBluePrint.get_attribute('image_size_y').as_int()
        # initialize camera data with those height and width 
        # np.zeros is numpy utility that Return a new array of given shape and type, filled with zeros.
        # we want array of rows equal to height, columns equal to width and we have 4 channels (red, green, blue and alpha)
        self.cameraDataDict = {'image' : np.zeros((self.cameraImageHeight, self.cameraImageWidth, 4))}
        # Camera listen call back
        self.camera.listen(lambda image: self._CameraCallback(image, self.cameraDataDict))

        # [debug setting the spectator]
        self.spectator = self.world.get_spectator()
        self.spectator.set_transform(self.camera.get_transform())
        
    
    def __applyCarlaSettings(self, DT):
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
        '''
        # Get world settings
        settings = self.world.get_settings()
        # change settings
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = DT
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = 10
        # Apply
        self.world.apply_settings(settings)

    
    def __configureIMUSensor(self):
        # Getting blue print
        IMUBluePrint = self.world.get_blueprint_library().find('sensor.other.imu')
        #[todo] : set blue print attr (such as std deviations and noise seeds)
        # getting IMU mouting point
        IMUMountingPoint = carla.Transform(carla.Location(z = 0, x=0))
        # Spawning the IMU and store it in the self pointer
        self.IMU = self.world.try_spawn_actor(IMUBluePrint, IMUMountingPoint, attach_to=self.car)
        # Configure listenning function
        self.IMU.listen(lambda data : self.__IMUListenCallBack(data))


    def __IMUListenCallBack(self, data):
        print("######### START DATA FROM IMU###########")
        print(data.accelerometer)
        print(data.frame)
        print(data.timestamp)
        print("#########END DATA FROM IMU###########")
    #------------
    ## Sensors Utilities
    #-------------
    # Method to open the front Camera OpenCvWindow
    def OpenCvFrontCameraInit(self):
        # Opening window named RGB Front Camera
        cv2.namedWindow('RGB Front Camera',cv2.WINDOW_AUTOSIZE)

    # Updatint openCv window view, need to be called periodically
    def OpenCvFrontCameraUpdate(self):
        image = self.cameraDataDict['image']
        # Adding the speed text to the winodow showing car camera
        image = cv2.putText(image, 'Speed: '+str(int(self.currentSpeed))+' kmh', self.OpenCVOrg2, self.OpenCVFont, self.OpenCVFontScale, self.OpenCVColor, self.OpenCVThickness, cv2.LINE_AA)
        # Showing data over the window
        cv2.imshow('RGB Front Camera',self.cameraDataDict['image'])
        # Wait for a small amount of time (1 millisecond) and check if a key is pressed
        # If no key is pressed, continue; otherwise, exit the loop
        cv2.waitKey(1)
        #[todo] : add parameters to openCv window


    #------------
    ## State Utilitites
    #-------------
    def SetCarState(self, desiredSpeed, desiredAngle):
        '''
            This is an initial function to control car state
            [todo] : do you need to pass sensor to the camera or use instance variables
        '''
        # Read velocity (which is vector in 3d) from the game in m/s
        currentVelocity = self.car.get_velocity()
        # converting velocity to speed, then converting m/s to km/h
        self.currentSpeed = (3.6 * math.sqrt(currentVelocity.x**2 +currentVelocity.y**2 +currentVelocity.z**2 ))

        #--------
        # for debugging
        #----------
        # self.speed_data.append(self.currentSpeed)
        # plt.plot(self.speed_data)
        # plt.xlabel('Time (s)')
        # plt.ylabel('speed')
        # plt.title('speed vs Time')
        # plt.grid(True)
        # plt.pause(0.001)  # Update the plot
        # DEBUGGING
        # print(self.currentSpeed)

        # Controlling the car throttle based on the current speed and desired speed 
        self.car.apply_control(carla.VehicleControl(throttle=self.__MaintainCarSpeed(desiredSpeed,self.currentSpeed), steer=0))

    # Private method for Maintaining the speed using Controller
    def __MaintainCarSpeed(self, desiredSpeed, currentSpeed):
        ''' 
        this is a very simple function to maintan desired speed
        s arg is actual current speed
        '''
        # Calculating Throttle openning percentage 
        throttleOpeningPercentage = self.throttlePID.update(setpoint=desiredSpeed, measurement=currentSpeed)
        # print(throttleOpeningPercentage)
        return throttleOpeningPercentage /100.0


    #------------
    ## Sensors Callbacks
    #-------------
    # Protected class method, Camera callback
    def _CameraCallback(self, image, dataDict):
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
                
    # Class destructor
    def __del__(self):
        # Destroying the car
        self.car.destroy()
        # Destroying the camera
        self.camera.destroy()
        # Closing all openCv2 windows
        cv2.destroyAllWindows() #[todo] : close specific opened windows
        # Stopping camera Sensor (method insied carla.Sensor)
        self.camera.stop()



def main():
    # Connecting to the server
    client = carla.Client('localhost',2000)
    # Get world Map
    world = client.get_world()
    carInstance = Car(client, world, 0.01)
    carInstance.OpenCvFrontCameraInit()


    ## Main loop
    while True:
        # applying world tick (synchronous mode)
        world.tick() 
        carInstance.OpenCvFrontCameraUpdate()
        carInstance.SetCarState(20,1)


if __name__ == '__main__':
    main()

