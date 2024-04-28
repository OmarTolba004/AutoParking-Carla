<p><a target="_blank" href="https://app.eraser.io/workspace/AYq08Xn2Ul6DKMjGkV2r" id="edit-in-eraser-github-link"><img alt="Edit in Eraser" src="https://firebasestorage.googleapis.com/v0/b/second-petal-295822.appspot.com/o/images%2Fgithub%2FOpen%20in%20Eraser.svg?alt=media&amp;token=968381c8-a7e7-472a-8ed6-4a6626da5501"></a></p>

# Vehicle_RaspberryPi_ECU
Repository for the vehicle RaspberryPi ecu system



Notes: 

side Notes: 

use carla actor velocity and angular velocity api and compare them to your actual velocity and angular velocity

Initialization Notes

1. xTrue initialization with position from simulation = [ [true x pos],[ture y pos],[ angle (zero by default)],[true velocity]]
2. xEst start need to be initialized with same values as xTrue
3. PEst can be eye matrix as initialization 


Input Calculation Notes: 

1. Input u gonna be calculated from IMU reading to form the u matrix -> Done
Observation Notes:

1- xTrue gonaa be updated using simulation to get new pos-x, pos-y 



<!--- Eraser file: https://app.eraser.io/workspace/AYq08Xn2Ul6DKMjGkV2r --->