import dartv2b
import time
import sys
import math
import pickle

if __name__ == "__main__":
    mybot = dartv2b.DartV2()

    spd_ang = 140
    try:
        spd_ang = int(sys.argv[1])
    except:
        pass

    duration = 10.0
    try:
        duration = float(sys.argv[2])
    except:
        pass
    
    accel_filter_mode = 2
    mybot.imu.setup_accel_filter(accel_filter_mode)

    
    mag = mybot.imu.read_mag_raw()
    magx_min = mag[0]
    magx_max = mag[0]
    magy_min = mag[1]
    magy_max = mag[1]
    tmag = []
    
    t0 = time.time()
    while True:
        if time.time()-t0 >= duration:
            break
        mybot.set_speed(-spd_ang,spd_ang)
        time.sleep(0.25)
        mybot.stop()
        for i in range(3):
            time.sleep(0.25)
            mag = mybot.imu.read_mag_raw()
            head = mybot.imu.heading_raw_deg(mag[0],mag[1])
            print (mag,head)
            tmag.append([mag,head])
            magx_min = mag[0] if mag[0]<magx_min else magx_min
            magx_max = mag[0] if mag[0]>magx_max else magx_max
            magy_min = mag[1] if mag[1]<magy_min else magy_min
            magy_max = mag[1] if mag[1]>magy_max else magy_max


    print ("min,max :", magx_min, magx_max, magy_min, magy_max)
    print ("Battery Voltage : %.2f V"%(mybot.encoders.battery_voltage()))
    mybot.stop() # stop motors
    mybot.end() # clean end of the robot mission

    pickle.dump(tmag,open("mag.log","wb"))
    # to get it : tmag=pickle.load(open("mag.log","rb"))

# DART 02 min,max : 902 3759 -3443 -737
# DART 02 min,max : 948 3894 -3510 -731
# DART 02 min,max : 915 3886 -3540 -713
# DART 04 min,max : -825 3088 -4548 -746
