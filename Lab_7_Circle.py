import time
import numpy as np
from Motor import *
PWM = Motor()

t = 0
time.sleep(3)
try:
    while True:
        if t <= 10:
            
            # constant speed
            v = 800
            omega = 600

            # speed control
            u = np.array([v - omega, v + omega])
            u[u > 1500] = 1500
            u[u < -1500] = -1500

            # Send control input to the motors
            PWM.setMotorModel(u[0], u[0], u[1], u[1])

            t = t + 0.1
            time.sleep(0.1)

# The usage is to interrupt the program with keyboard input
except KeyboardInterrupt:
    # This command will stop the motor
    PWM.setMotorModel(0,0,0,0)
    print ("\nProgram end because of keyboard interrupt")