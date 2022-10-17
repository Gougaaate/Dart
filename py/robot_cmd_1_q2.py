import dartv2b
import time

if __name__ == "__main__":
    mybot = dartv2b.DartV2()

    # place your work here
    print ("Front encoders before : ",mybot.get_front_encoders())
    
    mybot.set_speed (100,-100)
    time.sleep(2.0)
    mybot.set_speed (0,0)

    print ("Front encoders after : ",mybot.get_front_encoders())
    
    mybot.end() # clean end of the robot mission

