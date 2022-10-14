import rob1a_v02 as rob1a  # get robot simulator
import control
import time


if __name__ == "__main__":
    rb = rob1a.Rob1A()   # create a robot (instance of Rob1A class)
    ctrl = control.RobotControl() # create a robot controller


    spd_left = 10000 # define speed of left wheel
    spd_right = 10000 # define speed of right wheel
    duration = 100 # move for 2 seconds
    ctrl.test_move(rb,spd_left,spd_right,duration)

    rb.full_end() # clean end of the simulation
