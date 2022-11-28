import drivers_v2.drivers_v2 as drv2
import time

if __name__ == "__main__":
    mybot = drv2.DartV2DriverV2()

    # place your work here
    print ("Sonar Microcode Version : ",mybot.sonars.get_version())
    print ("Battery Voltage : %.2f V"%(mybot.encoders.battery_voltage()))

    # set range max to 1.5 m
    dmax = 1.5
    for isn in range(4):
        mybot.sonars.set_dist_max(isn+1,2.0)
    mybot.sonars.set_dist_max(5,dmax)

    # test mode 2 on 4 sonars
    print ("test mode 2 (sync)  ...")
    mode = 2
    for isn in range(4):
        mybot.sonars.set_mode(isn+1,mode)
    for itst in range(10): 
        time.sleep(0.1)
        print ("Sonars cardinal",mybot.sonars.read_4_sonars())

    # test mode 1 on front sonar
    print ("test mode 1 (one shot) on front and rear ...")
    mode = 0   # stop all
    for isn in range(4):
        mybot.sonars.set_mode(isn+1,mode)
    mode = 1
    for itst in range(10): 
        for isn in range(4):
            mybot.sonars.set_mode(isn+1,mode)
        time.sleep(0.5)
        print ("Sonars front,rear,left,right",mybot.sonars.read_front(),mybot.sonars.read_rear(),mybot.sonars.read_left(),mybot.sonars.read_right())    
    mybot.end() # clean end of the robot mission

