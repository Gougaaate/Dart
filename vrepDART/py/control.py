import numpy as np
import time

class RobotControl:
    def __init__(self):
        # set some useful constants
        self.distBetweenWheels = 0.12
        self.nTicksPerRevol = 512
        self.wheelDiameter = 0.06


    def test_move (self,rb,speed_left,speed_right,duration):
        """
        Example of test function to check if the robot moves

        input parameters :
        rb : robot object
        speed_left : speed command of left wheel
        speed_right : speed command of right wheel
        duration : duration of the move

        output paremeters :
        None
        """
        # forward motion
        rb.set_speed(speed_left,speed_right)
        loopIterTime = 0.050
        tStart = time.time()
        while time.time()-tStart < duration:
            time.sleep(loopIterTime) # wait 
        # stop the robot 
        rb.stop()