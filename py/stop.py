import dartv2b
import time
import sys

if __name__ == "__main__":
    mybot = dartv2b.DartV2()

    # speed difference to get same count on left and right odo
    spdl = 0
    spdr = 0
    mybot.set_speed (spdl, spdr)
    time.sleep (0.05)

    print ("Battery Voltage : %.2f V"%(mybot.encoders.battery_voltage()))
   
    mybot.end() # clean end of the robot mission

