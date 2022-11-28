import drivers_v2.drivers_v2 as drv2
import time

if __name__ == "__main__":
    mybot = drv2.DartV2DriverV2()

    # place your work here
    print ("Front encoders before : ",mybot.powerboard.get_front_encoders())
    
    mybot.powerboard.set_speed (100,-100)
    time.sleep(2.0)
    mybot.powerboard.set_speed (0,0)

    print ("Front encoders after : ",mybot.powerboard.get_front_encoders())
    
    mybot.end() # clean end of the robot mission

