"""************************************************************************************************************************
*   File Name: sim.py
*   Authors: Omar Tolba
*   Last modifing Date: 14/6/2024
***************************************************************************************************************************
*   Description: simulation main file
************************************************************************************************************************"""
import carla
from Car import Car

def main():
    # Connecting to the server
    client = carla.Client('localhost',2000)
    # Get world Map
    world = client.get_world()
    carInstance = Car(client, world, 0.01)
    carInstance.OpenCvFrontCameraInit()
    carInstance.initialize_path_tracking(carInstance.spawnPoints[0].location, carInstance.spawnPoints[-49].location)
    ## Main loop
    while True:
        # applying world tick (synchronous mode)
        world.tick() 
        carInstance.execCounter +=1
        carInstance.OpenCvFrontCameraUpdate()
        # carInstance.SetCarState(80,1)
        
        if(carInstance.execCounter < 10):
            continue
        carInstance.apply_sensor_fusion()
        carInstance.apply_path_tracking()
        


if __name__ == '__main__':
    main()
