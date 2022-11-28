import drivers_v2.drivers_v2 as drv2
import dartv2_control 
import time

if __name__ == "__main__":
    mybot = drv2.DartV2DriverV2()
    mybot_ctrl = dartv2_control.DartV2Control(mybot)

    # place your work here
    print ("Front encoders before : ",mybot_ctrl.get_front_encoders(init=True))

    for ileg in range(2):
        mybot.powerboard.set_speed (100, 100)
        time.sleep(2.0)
        mybot.powerboard.set_speed (100,-100)
        time.sleep(1.33) # empirical !! may change with cpu !!! 
        mybot.powerboard.set_speed (0,0)

    odo_left,odo_right = mybot_ctrl.get_front_encoders()
    
    print ("Front encoders after : ",[odo_left,odo_right])
    # save memory before delta computation that sets the memory to the last value
    olo_left, odo_right = mybot_ctrl.front_encoders_get_memory()
    deltaOdoLeft = mybot_ctrl.delta_front_odometers(side="left")
    deltaOdoRight = mybot_ctrl.delta_front_odometers(side="right")
    print ("Delta odometer left :", deltaOdoLeft)
    print ("Delta odometer right :", deltaOdoRight)

    # restore the memory to compute delta again (with both)
    mybot_ctrl.front_encoders_set_memory(olo_left, odo_right )
    # deltas qhould be identical
    print ("Delta odometers :",mybot_ctrl.delta_front_odometers())
    
    mybot.end() # clean end of the robot mission

