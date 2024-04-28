import carla
from Car import Car

def main():
    # Connecting to the server
    client = carla.Client('localhost',2000)
    # Get world Map
    world = client.get_world()
    carInstance = Car(client, world, 0.01)
    carInstance.configureSensors()
    carInstance.OpenCvFrontCameraInit()


    ## Main loop
    while True:
        # applying world tick (synchronous mode)
        world.tick() 
        carInstance.execCounter +=1
        carInstance.OpenCvFrontCameraUpdate()
        carInstance.SetCarState(20,1)
        carInstance.applySensorFusion()
        



if __name__ == '__main__':
    main()
