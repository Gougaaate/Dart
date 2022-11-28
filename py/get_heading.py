import sys
sys.path.insert(0, "../drivers_v2")
sys.path.insert(0, "../vDartV2")
import drivers_v2 as drv2
import time
import sys
import math

def delta_heading(head_ref,head):
    head_err = head_ref-head
    while head_err > 180.0:
        head_err -= 360.0
    while head_err <= -180.0:
        head_err += 360.0
    #print ("heading error",head_ref,head,head_ref-head,head_err)
    return head_err


if __name__ == "__main__":
    mybot = drv2.DartV2DriverV2()

    duration = 1.0
    try:
        duration = float(sys.argv[2])
    except:
        pass

    dt = 0.1
    try:
        dt = float(sys.argv[2])
    except:
        pass


    # set fast mag calib data DART02
    magx_min = 902
    magx_max = 3894
    magy_min = -3510
    magy_max = -731
    # set fast mag calib data DART04
    magx_min = -825
    magx_max = 3088
    magy_min = -4548
    magy_max = -746    

    # DART02 - 2021/12/2
    magx_min = 779
    magx_max = 4088
    magy_min = -3622
    magy_max = -553

    # DART05 - 2022/09/23
    magx_min = -187
    magx_max = 2207
    magy_min = -2035
    magy_max = 234

    # DART06 - 2022/11/25
    magx_min = 957
    magx_max = 4933
    magy_min = -6478
    magy_max = -2659

    # Simulated DART
    #magx_min = 0
    #magx_max = 1000
    #magy_min = 0
    #magy_max = 1000

    mybot.imu.fast_heading_calibration (magx_min, magx_max, magy_min, magy_max)

    print ("Battery Voltage : %.2f V"%(mybot.encoders.battery_voltage()))

    t0 = time.time()
    head_err_i = 0.0
    while True:
        if time.time()-t0 >= duration:
            break
        mag = mybot.imu.read_mag_raw()
        print ("heading : %.2f"%(mybot.imu.heading_deg(mag[0],mag[1])))
    print ("Battery Voltage : %.2f V"%(mybot.encoders.battery_voltage()))

    mybot.powerboard.stop() # stop motors
    mybot.end() # clean end of the robot mission
