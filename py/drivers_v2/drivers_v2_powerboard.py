#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import sys
import os

class PowerBoardIO():
    def __init__(self,exec_robot,vsv=None):
        self.__vSimVar = vsv
        self.__exec_robot = exec_robot
        print ("Exec Robot",exec_robot)
        if exec_robot == "Sim V-REP":
            import drivers_v2_trex as trex_drv
        elif exec_robot == "Sim GAZEBO":
            pass
        elif exec_robot == "Real":
            sys.path.append('./drivers_v2')
            import drivers_v2_trex as trex_drv
        elif exec_robot == "Real ROS":
            import dartv2_drivers.drivers.drivers_v2_trex as trex_drv
        self.__trex = trex_drv.TrexIO(exec_robot=exec_robot)
        self.__trex.command["use_pid"] = 0 # 1 
        self.speed_acq_time = time.time()
        self.speed_left_last = 0
        self.speed_right_last = 0

    def stop(self):
        self.set_speed(0,0) # stop motors
                    
    def set_speed(self,speedLeft,speedRight):
        if speedLeft > 255:
            speedLeft = 255
        if speedLeft < -255:
            speedLeft = -255
        if speedRight > 255:
            speedRight = 255
        if speedRight < -255:
            speedRight = -255
        if self.__exec_robot == "Sim V-REP":
            #print (self,speedLeft,speedRight)
            # bug corrected 2020/12/01, CmdSpeedNew must be 0.0 to
            # take into account the new command (loop until 0.0)
            while self.__vSimVar["vCmdSpeedNew"] != 0.0:
                time.sleep (0.001)
            #print ("simvar before",self.__vSimVar["vCmdSpeedLeft"],self.__vSimVar["vCmdSpeedRight"],self.__vSimVar["vCmdSpeedNew"])
            self.__vSimVar["vCmdSpeedLeft"] = float(speedLeft)
            self.__vSimVar["vCmdSpeedRight"] = float(speedRight)
            self.__vSimVar["vCmdSpeedNew"] = 1.0
            #print ("simvar after",self.__vSimVar["vCmdSpeedLeft"],self.__vSimVar["vCmdSpeedRight"],self.__vSimVar["vCmdSpeedNew"])
            if speedLeft >=0:
                self.__vSimVar["vMotorRearDirectionLeft"] = 0
            else:
                self.__vSimVar["vMotorRearDirectionLeft"] = 1
            if speedRight >=0:
                self.__vSimVar["vMotorRearDirectionRight"] = 0
            else:
                self.__vSimVar["vMotorRearDirectionRight"] = 1
        else:
            self.__trex.command["left_motor_speed"] = int(speedLeft)
            self.__trex.command["right_motor_speed"] = int(speedRight)
            self.__trex.i2c_write()
        self.speed_left_last = int(speedLeft)
        self.speed_right_last = int(speedRight)
        self.speed_acq_time = time.time()

    def get_front_encoders(self):
        if self.__exec_robot == "Sim V-REP":
            encFrontLeft = self.__vSimVar["vEncoderFrontLeft"]
            encFrontRight = self.__vSimVar["vEncoderFrontRight"]
        else:
            status = self.__trex.status
            #print ("status",status)
            encFrontLeft = status["left_encoder"]
            encFrontRight = status["right_encoder"]
        return [encFrontLeft,encFrontRight]

if __name__ == "__main__":
    # warning, tests are quite complex in simulation as we need to connect
    # the module to the V-REP simulator...

    # test if on real robot , if gpio exists (not very robust)
    # add test on processor type, real robot has armv7l
    tstsim = False
    if (os.access("/sys/class/gpio/gpio266", os.F_OK)) \
       and (platform.processor() == 'armv7l'):
        encoders = EncodersIO("Real")
        print ("Work with real DART")
    # if not the virtual robot is running in V-REP
    else :
        tstsim = True
        sys.path.append('../vDartV2')
        import vSimVar as vsv
        tSimVar= vsv.tSimVar
        powerboard = PowerBoardIO("Sim V-REP",vsv=tSimVar)
        # initiate communication thread with V-Rep
        tSimVar["vSimAlive"] = False
        import vrep_interface as vrep
        vrep_itf = vrep.VrepInterface(tSimVar)
        vrep = vrep_itf.start_thread()
        print ("vDart ready ...")
        print ("Simulation alive is ",tSimVar["vSimAlive"])
        print ("Work with virtual DART on V-REP")

    try:
        val_left = int(sys.argv[1])
    except:
        val_left = 80
    
    try:
        val_right = int(sys.argv[2])
    except:
        val_right = -80
    
    try:
        duration = float(sys.argv[3])        
    except:
        duration = 1.0 
    powerboard.set_speed(val_left,val_right)
    time.sleep(duration)
    powerboard.stop()


    if tstsim:
        time.sleep(0.25)
        tSimVar["vSimAlive"] = False
        time.sleep(0.25)
    
