import carla
import time
import random 
import numpy as np


'''
Todos:
    1- [Todo] : control map layers, load and unload map layer using world object
    2- [Todo] : Control car lights when applying specifing steering angle
    3- [Todo] : work in synchronous mode
    4- [Todo] : customizing the weather
    5- [Todo] : destroying all sensors and cv2 cleanup
    6- [todo] : load specific town at the begining 
'''
class Car:

    # ----------------
    ## Class Variables
    # ----------------
    # Camera offset on the Z and X direction
    CAMERA_POS_Z = 1
    CAMERA_POS_X = 0.4
    # Sensors Data Dictionary 
    # dataDict = {} [Todo]

    # ----------------
    ## Class Constructor
    # ----------------
    def __init__(self):

        # ----------------
        ## Setting up the world and the vehicle
        # ----------------
        # Connecting to the server
        self.client = carla.Client('localhost',2000)
        # Get world Map
        self.world = self.client.get_world()
        # Get all avialable spawn points
        self.spawnPoints = self.world.get_map().get_spawn_points()
        # Getting start point of the Car
        self.startPoint = self.spawnPoints[0]
        # Getting specific Car blueprint
        self.carBluePrint = self.world.get_blueprint_library().find('vehicle.lincoln.mkz_2020')
        # Setting the spawned Car as the ego vehicle and specifying the color
        self.carBluePrint.set_attribute('role_name', 'hero')
        self.carBluePrint.set_attribute('color', '0 ,0 ,0')
        # Trying to spawn the car in the start point
        self.car = self.world.try_spawn_actor(self.carBluePrint, self.startPoint) 
        # Checking if car spawned correctly 
        if(self.car is None) :
            print("Couldn't spawn the lincoln mkz 2020 car")
            exit()

        # ----------------
        ## Setting Sensors
        # ----------------
        
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
        
        # Getting camera image width and height to initalized dict key value pair with
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
        
        time.sleep(10)
    
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
        
        print("Entered CallBack successfully ") # [todo]: remove this line
        
    # Class destructor
    def __del__(self):
        # Destroying the car
        self.car.destroy()
        # Destroying the camera
        self.camera.destroy()



dummy = Car()