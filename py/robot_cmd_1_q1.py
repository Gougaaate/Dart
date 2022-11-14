import dartv2b
import time

if __name__ == "__main__":
    mybot = dartv2b.DartV2()
    mybot.end() # clean end of the robot mission


    mybot.set_speed (100,-100)
    time.sleep(2.0)
    mybot.set_speed (0,0)
    