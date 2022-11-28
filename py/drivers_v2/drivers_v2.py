#!/usr/bin/python
# -*- coding: utf-8 -*-

# main driver : manage 4 cases :
#   - real robot 
#   - simulated robot V-REP (or CoppeliaSim)
#   - real robot with ROS
#   - simulated robot with GAZEBO

import signal
import sys
import time
import os
import platform

class DartV2DriverV2 ():
    def __init__(self, exec_robot="Auto", debug=False):
        self.__exec_robot = exec_robot
        print ("Init Dartv2b Driver V2 in exec_robot ",self.__exec_robot)
        self.__debug = debug
        self.__dartSimGazebo = False      
        self.__dartSimVrep = False
        self.__dartRos = False
        self.__dartReal = False        
        if exec_robot == "Sim V-REP":    
            self.__dartSimVrep = True
        elif exec_robot == "Sim GAZEBO":
            self.__dartSimGazebo = True      
        elif exec_robot == "Real":
            self.__dartReal = True
        elif exec_robot == "Real ROS":           
            self.__dartRos = True       
        elif exec_robot == "Auto":
            # check if on real robot
            # if yes : set exec_robot to "Real"
            # if no : set exec_robot to "Sim V-REP"
            if (os.access("/sys/class/gpio/gpio266", os.F_OK)) \
            and (platform.processor() == 'armv7l'):
                exec_robot = "Real"
                self.__dartReal = True
            else:
                exec_robot = "Sim V-REP"
                self.__dartSimVrep = True
            self.__exec_robot = exec_robot
        self.__sim = self.__dartSimVrep or self.__dartSimGazebo  
        self.__dartSim =  self.__sim
        print ("Init Dartv2b Driver V2 in exec_robot ",self.__exec_robot)

        self.__vSimVar = None
        # self.__vrep_itf = None
        # self.__vrep =None

        # trap hit of ctrl-x to stop robot and exit (emergency stop!)
        signal.signal(signal.SIGINT, self.interrupt_handler)
        
        if not self.__dartSim:
            # test if on real robot , if gpio exists (not very robust)
            # add test on processor type, real robot has armv7l
            if (os.access("/sys/class/gpio/gpio266", os.F_OK)) \
                and (platform.processor() == 'armv7l'):
                print ("Work with real DART")
            else:
                print ("Cannot access real DART ... exit")
                exit(0)
        elif self.__dartSimVrep:
            # check that V-REP is running and initialize the link
            import psutil
            found = False
            for pr in psutil.process_iter():
                if pr.name() == "vrep":
                    found = True
                    break
                if pr.name() == "coppeliaSim":
                    found = True
                    break                        
            if not found:
                print ("V-REP (or CoppeliaSim) is not running ... exit")
                exit(0)
            #print ("Python Sys Path",sys.path)
            sys.path.append('../vDartV2')
            sys.path.append('./vDartV2')
            #print ("Python Sys Path Updated",sys.path)
            # virtual DART requires multiprocessing and sockets
            import threading
            import socket
            import struct
            # global simvar can be accessed with vsv.tSimVar dict
            # eg simTime = vsv.tSimVar["vSimTime"]
            import vSimVar as vsv
            self.__vSimVar = vsv.tSimVar
            self.__vSimVar["vSimAlive"] = False
            import vrep_interface as vrep
            self.__vrep_itf = vrep.VrepInterface(self.__vSimVar,self.__debug)
            # initiate communication thread with V-Rep
            self.__vrep = self.__vrep_itf.start_thread()
            print ("vDart ready ...")
            #print ("Simulation alive is ",self.__vSimVar["vSimAlive"])
        elif self.__dartSimGazebo:
            # check that Gazebo is running (todo)
            pass

        #print ("Python Sys Path",sys.path)
        sys.path.append('./drivers_v2')
        #print ("Python Sys Path Updated",sys.path)
        # Import modules to load the drivers
        from drivers_v2_powerboard import PowerBoardIO 
        from drivers_v2_sonars import SonarsIO 
        from drivers_v2_encoders import EncodersIO 
        from drivers_v2_imu9 import Imu9IO 
        self.powerboard = PowerBoardIO(self.__exec_robot, vsv=self.__vSimVar)
        self.sonars = SonarsIO(self.__exec_robot, vsv=self.__vSimVar)
        self.encoders = EncodersIO(self.__exec_robot, vsv=self.__vSimVar)
        self.imu = Imu9IO(self.__exec_robot, vsv=self.__vSimVar)

    def dart_sim(self):
        return self.__dartSim

    def dart_ros(self):
        return self.__dartRos

    def exec_robot(self):
        return self.__exec_robot

    # clean stop of simulation
    def end(self):
        self.powerboard.stop() # stop motors
        if self.__dartSimVrep:        
            #print (self.__vSimVar)
            for isn in range(6):
                sonar_num = isn+1
                kySimEnd = "vSonar%1dSimEnd"%(sonar_num)
                #print ("stop sonar",sonar_num)
                self.__vSimVar[kySimEnd] = True   
            #print (self.sonars.vsv)
            for isn in range(6):
                sonar_num = isn+1
                kySim = "vSonar%1dSim"%(sonar_num)
                while True:
                    #print ("check for end, sonar",sonar_num)
                    if not self.__vSimVar[kySim]:
                        break
                    #print ("wait for end, sonar",sonar_num)
                    time.sleep(1.0)
                    pass
                #print ("sonar",sonar_num," ended ...")
            self.__vSimVar["vSimAlive"] = False
            #print ("Simulation alive is ",self.__vSimVar["vSimAlive"])
           
            time.sleep(0.1)
            print ("... end")           

    def simulation_time(self):
        simTime = time.time()
        if self.__dartSim:
            simTime = self.__vSimVar["vSimTime"]
        return simTime

    # interrupt handler
    def interrupt_handler(self,signal, frame):
        print ('You pressed Ctrl+C! DART V2 will stop immediately')
        #self.powerboard.stop() # stop motors (now done in end())
        self.end() # clean stop of simulator
        sys.exit(0)

if __name__ == "__main__":
    drv2 = DartV2DriverV2 ()
    for i in range(1):
        time.sleep(1.0)
    drv2.end()
