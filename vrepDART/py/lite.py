import rob1a_v02 as rob1a  # get robot simulator
import control  # robot control functions
import filt # sensors filtering functions
import numpy as np
import time


if __name__ == "__main__":
    pseudo = ""  # you can define your pseudo here
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)
    ctrl = control.RobotControl() # create a robot controller

    # put your mission code here

    rb.full_end()
