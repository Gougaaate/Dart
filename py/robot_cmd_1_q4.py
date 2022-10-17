import dartv2b
import time

if __name__ == "__main__":
    mybot = dartv2b.DartV2()

    # place your work here
    print ("Front encoders before : ",mybot.get_front_encoders())

    for ileg in range(2):
        mybot.set_speed (100, 100)
        time.sleep(5.0)
        mybot.set_speed (100,-100)
        time.sleep(1.35) # empirical !! may change with cpu !!! 
        mybot.set_speed (0,0)

    odo_left,odo_right = mybot.get_front_encoders()
    
    print ("Front encoders after : ",[odo_left,odo_right])
    deltaOdoLeft = mybot.delta_front_odometers(side="left")
    deltaOdoRight = mybot.delta_front_odometers(side="right")
    print ("Delta odometer left :", deltaOdoLeft)
    print ("Delta odometer right :", deltaOdoRight)
    print ("Delta odometers :",mybot.delta_front_odometers())
    
    mybot.end() # clean end of the robot mission

