"""************************************************************************************************************************
*   File Name: pidCPython.py
*   Authors: Omar Tolba
*   Last modifing Date: 14/4/2024
***************************************************************************************************************************
*   Description: python wrapper for pid C implementation
************************************************************************************************************************"""
import ctypes

# Define the structure PIDController_t using ctypes
class PIDController_t(ctypes.Structure):
    _fields_ = [
        ("controllerType", ctypes.c_int),  # Assuming PIDControllerType_t is an integer type
        ("kp", ctypes.c_float),
        ("ki", ctypes.c_float),
        ("kd", ctypes.c_float),
        ("T", ctypes.c_float),
        ("maxLimit", ctypes.c_float),
        ("minLimit", ctypes.c_float),
        ("out", ctypes.c_float),
        ("integrator", ctypes.c_float),
        ("prevError", ctypes.c_float),
        ("maxLimitInt", ctypes.c_float),
        ("minLimitInt", ctypes.c_float),
        ("differentiator", ctypes.c_float),
        ("prevMeasurement", ctypes.c_float),
        ("tau", ctypes.c_float)
    ]

#Class for wrapping C implenation PID to python

class pid:
    #--------
    # Class Variables
    #------------
    #Controller types
    # P = 0,  
    # PI = 1, 
    # PD = 2, 
    # PID = 3 

    def __init__(self, l_kp = 2.0, l_ki= 0.001, l_kd= 0.0,l_T= 0.01, l_maxLimit= 10.0, l_minLimit= -10.0, l_maxLimitInt= 5.0, l_minLimitInt= -5.0, l_tau= 0.02):
        # loading the library
        self.lib = ctypes.CDLL("/home/omar/gp/Mycode/libPID.so") 
        # inform python of the init function arguments
        # This part of the line assigns a list to the argtypes attribute. The list contains the types of the arguments that the PIDController_Init function expects. In this case.
        # it indicates that the function expects a single argument, which is a pointer to a PIDController_t structure.
        self.lib.PIDController_Init.argtypes = [ctypes.POINTER(PIDController_t)]
        # Inform python of Init function return type
        self.lib.PIDController_Init.restype = None
        # Initialize PIDController_t structure with values
        self.myPid = PIDController_t(
            controllerType= 3,  # Forced to be PID
            kp= l_kp,
            ki= l_ki,
            kd= l_kd,
            T= l_T,
            maxLimit= l_maxLimit,
            minLimit= l_minLimit,
            maxLimitInt= l_maxLimitInt,
            minLimitInt= l_minLimitInt,
            tau= l_tau #Derivative (band-limited differentiator) low-pass filter time constant 
        )
        #Call PIDController Init function 
        self.lib.PIDController_Init(ctypes.byref(self.myPid))

    def update(self, setpoint, measurement) -> float:
        # Placeholder for the output value
        self.output = ctypes.c_float()
        # Calculating the output
        self.lib.PIDController_Update(ctypes.byref(self.output), ctypes.byref(self.myPid), ctypes.c_float(setpoint), ctypes.c_float(measurement))
        return self.output.value
