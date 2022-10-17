#!/usr/bin/python
# -*- coding: utf-8 -*-

# Second Year ENSTA Bretagne SPID Project
#    D : Brian Detourbet
#    A : Elouan Autret
#    A : Fahad Al Shaik
#    R : Corentin Rifflart
#    R : Clément Rodde
#    T : Rémi Terrien

# Code modified in May 2016, by BenBlop (small changes to conform with
#  new DART)

# Code modified in May 2017, by BenBlop (big changes to adapt to Irvin's
# drivers and to communicate with V-REP using sockets

# Code modified in April 2018 to adapt to Dart V2 (BenBlop)
#    4 sonars changed (accessed with I2C)
#    2 diagonal sonars added (I2C)
#    Razor IMU (serial) changed to POLOLU IMU (I2C)
#    7 segments display added (I2C)
#    Front encoders added (I2C)

# Code modified in Summer 2019 to use new microcode on DART microcontrollers
#    change in sonars and seven segments display
# code taken from : /home/rob/Teach/old-vrep-sims/robmob-dartv2-code-2019/tests/dartv2.py

# v-rep 3.3.2, path :
# ./V-REP_PRO_EDU_V3_3_2_64_Linux
# 2017 simulator build path :
# /home/newubu/Lecture/robmob/dart/Simulateur/build/2017-05-10

# new version allowing to run the same code on real and virtual DARTV2
#  - server is now on the lua side
#  - no need to have to driver code , one for sim and one for real
#    e.g. before vSonars.py was used on the sim and sonars.py on the real robot
#    now sonars.py works for both sim and real
#  - I2C bus simulation in i2csim.py uses a dict with all alllowed I2C commands
#       the dict contains the function to execute on the sim 




import os
import time
import math
import sys
import signal
import random
import pickle
import platform

class DartV2Basis():
    # interrupt handler
    def interrupt_handler(self,signal, frame):
        print ('You pressed Ctrl+C! DART V2 will stop immediatly')
        self.stop() # stop motors
        self.end() # clean stop of simulator
        sys.exit(0)
            
    def __init__(self):
        print ("Init Dartv2b Basis")
        self.__dartSim = False
        self.__debug = False        

        # trap hit of ctrl-x to stop robot and exit (emergency stop!)
        signal.signal(signal.SIGINT, self.interrupt_handler)
        
        # test if on real robot , if gpio exists (not very robust)
        # add test on processor type, real robot has armv7l
        if (os.access("/sys/class/gpio/gpio266", os.F_OK)) \
           and (platform.processor() == 'armv7l'):
            print ("Work with real DART")
        # if not the virtual robot is running in V-REP
        else :
            self.__dartSim = True
            print ("Work with virtual DART on V-REP")
            
        #print ("Python Sys Path",sys.path)
        sys.path.append('./drivers')
        if self.__dartSim:
            sys.path.append('../vDartV2')
        #print ("Python Sys Path Updated",sys.path)

        # virtual DART requires multiprocessing and sockets
        self.__vSimVar = None
        if self.__dartSim:
            import threading
            import socket
            import struct
            # global simvar can be accessed with vsv.tSimVar dict
            # eg simTime = vsv.tSimVar["vSimTime"]
            sys.path.append('../vDartV2')
            import vSimVar as vsv
            self.__vSimVar = vsv.tSimVar
            self.__vSimVar["vVoltageBin"] = self.battery_voltage_v2bin(self.__vSimVar["vVoltage"])
                        
        # Import modules to load the drivers
        from drivers.trex import TrexIO
        from drivers.sonars import SonarsIO
        from drivers.encoders import EncodersIO
        from drivers.imu9 import Imu9IO
        self.__trex = TrexIO(sim=self.__dartSim)
        #print (self.__vSimVar)
        self.sonars = SonarsIO(sim=self.__dartSim, vsv=self.__vSimVar)
        self.encoders = EncodersIO(sim=self.__dartSim, vsv=self.__vSimVar)
        self.__trex.command["use_pid"] = 0 # 1
        self.imu = Imu9IO(sim=self.__dartSim, vsv=self.__vSimVar)

        # initiate communication thread with V-Rep
        if self.__dartSim:
            self.dtmx = -1.0
            self.dtmn = 1e38
            self.cnt_sock = 0
            self.dta_sock = 0.0
            self.upd_sock = False
            import threading
            import socket
            self.vdart_ready = threading.Event()
            self.vdart_ready.clear()
            # socket connection to V-REP
            self.__HOST = 'localhost'  # IP of the sockect
            self.__PORT = 30100 # port (set similarly in v-rep)
            self.server_address = (self.__HOST, self.__PORT)
            srv = self.server_address
            ev = self.vdart_ready
            self.__simulation_alive = True
            self.vrep = threading.Thread(target=self.vrep_com_socket,args=(srv,ev,))
            self.vrep.start()

            # wait for vdart to be ready
            self.vdart_ready.wait()
            print ("vDart ready ...")

        # place your new class variables here
        
        self.encoders_front_acq_time = time.time()
        self.encoders_front_left_last = 0  # last value
        self.encoders_front_right_last = 0
        self.encoders_front_left_mem = 0 # memory value 
        self.encoders_front_right_mem = 0
        enc_fr = self.get_front_encoders() # init last value

        self.encoders_rear_acq_time = time.time()
        self.encoders_rear_left_last = 0  # last value
        self.encoders_rear_right_last = 0
        self.encoders_rear_left_mem = 0 # memory value 
        self.encoders_rear_right_mem = 0
        enc_rr = self.get_rear_encoders() # init last value

        self.sonars_cardinal_acq_time = time.time()
        self.sonars_cardinal_front = 99.9
        self.sonars_cardinal_left = 99.9
        self.sonars_cardinal_rear = 99.9
        self.sonars_cardinal_right = 99.9

        self.sonars_diagonal_acq_time = time.time()
        self.sonars_diagonal_left = 99.9
        self.sonars_diagonal_right = 99.9
         
        self.speed_acq_time = time.time()
        self.speed_left_last = 0
        self.speed_right_last = 0

    def dart_sim(self):
        return self.__dartSim

    def simulation_time(self):
        simTime = time.time()
        if self.__dartSim:
            simTime = self.__vSimVar["vSimTime"]
        return simTime

    def vrep_com_socket(vdart,srv,ev):
        import socket
        import struct
        # setup com format
        comDataSize = 0
        comData = []
        comFmt = ""
        #vDoLog
        comFmt += "f"
        comDataSize += 4
        #vCmdSpeedLeft
        comFmt += "f"
        comDataSize += 4
        #vCmdSpeedRight
        comFmt += "f"
        comDataSize += 4
        #vCmdSpeedNew
        comFmt += "f"
        comDataSize += 4
        #vEncoderRearLeftReset
        comFmt += "f"
        comDataSize += 4
        #vEncoderRearRightReset
        comFmt += "f"
        comDataSize += 4

        comFullFmt = '<BBHH'+comFmt  # HH for data size in bytes
        comDataSizeLow = comDataSize % 256
        comDataSizeHigh = comDataSize // 256

        # rx data :
        # hd0,hd1,sz,lft
        # simulationTime
        # vSonarFront, vSonarRear,vSonarLeft, vSonarRight
        # vSonarFrontLeft,vSonarFrontRight
        # vEncoderFrontLeft, vEncoderFrontRight, vEncoderRearLeft, vEncoderRearRight
        # vXRob,vYRob,vZRob,vXAcc,vYAcc,vZAcc,vXGyro,vYGyro,vZGyro
       
        
        while True:
            vDoLog = vdart.__vSimVar["vDoLog"]
            vCmdSpeedLeft = vdart.__vSimVar["vCmdSpeedLeft"]
            vCmdSpeedRight = vdart.__vSimVar["vCmdSpeedRight"]
            vCmdSpeedNew = vdart.__vSimVar["vCmdSpeedNew"]
            vEncoderRearLeftReset = vdart.__vSimVar["vEncoderRearLeftReset"]
            vEncoderRearRightReset = vdart.__vSimVar["vEncoderRearRightReset"]
            #print ("update vrep with sock")
            #print (rob1a.simulation_alive,rob1a.speedLeft,rob1a.speedRight)
            # Create a TCP/IP socket
            t0 = time.time()
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                sock.connect(srv)
            except:
                print ("Simulation must be alive to execute your python program properly.")
                print ("Type Ctrl-C to exit, start the simulation and then execute your python program.")
                break

            sock.settimeout(0.5)
            #print (comFullFmt,ord(chr(59)),ord(chr(57)),
            #       comDataSizeLow,comDataSizeHigh,
            #       vDoLog,vCmdSpeedLeft,vCmdSpeedRight,vCmdSpeedNew,
            #       vEncoderRearLeftReset,vEncoderRearRightReset)
            strSend = struct.pack(comFullFmt,ord(chr(59)),ord(chr(57)),
                                  comDataSizeLow,comDataSizeHigh,
                                  vDoLog,vCmdSpeedLeft,vCmdSpeedRight,vCmdSpeedNew,vEncoderRearLeftReset,vEncoderRearRightReset) 
            sock.sendall(strSend)
            upd_sock = True

            data = b''
            try:
                while len(data) < 6:
                    data += sock.recv(200)
            except:
                print ("socker error , duration is %f ms, try to reconnect !!!"%((time.time() - t0)*1000.0))
                #sock.detach()
                #sock.connect(srv)
                #print ("socker error , type Ctrl-C to exit !!!")
                #exit(0)

            #print (len(data))
            rxHeader = struct.unpack('<ccHH',data[0:6])
            rxDataSize = 6 + rxHeader[2] + 256*rxHeader[3]
            while True:
                data_nw = sock.recv(2)
                if len(data_nw) == 0:
                    #print (len(data_nw))
                    break
                else:
                    data += data_nw
                    #print (len(data))
            #print (rxHeader,rxDataSize,len(data))
            if len(data) >= rxDataSize:
                vrx = struct.unpack('<ccHHfffffffffffffffffffffff',data[0:rxDataSize])
                vdart.vrep_update_sim_param(upd_sock,vrx[4:])
            else:
                print ("bad data length ",len(data))

            sock.close()
            vdart.cnt_sock = vdart.cnt_sock + 1
            tsock = (time.time() - t0)*1000.0
            vdart.dta_sock += tsock
            if tsock > vdart.dtmx:
                vdart.dtmx = tsock
            if tsock < vdart.dtmn:
                vdart.dtmn = tsock
            dtm = vdart.dta_sock/float(vdart.cnt_sock)
            #print ("tsock",tsock)
            if vdart.__debug:
                if (vdart.cnt_sock % 100) == 99:
                    print ("min,mean,max socket thread duration (ms) : ",vdart.dtmn,dtm,vdart.dtmx)

            #time.sleep(0.2)
            #print (dir(ev))
            
            if vCmdSpeedNew != 0.0:
                vdart.__vSimVar["vCmdSpeedNew"] = 0.0 # reset motor change cmd 
            ev.set()

            #print (vdart.__simulation_alive)
            if not vdart.__simulation_alive:
                break 

    # update parameters (from new vales given by V-REP simulator)
    def vrep_update_sim_param(self,upd_sock,vrx):
        #print (upd_sock)
        self.upd_sock = upd_sock

        #print ("vrx",vrx)
        simulationTime = vrx[0]

        vSonarFront = vrx[1]
        vSonarRear = vrx[2]
        vSonarLeft = vrx[3]
        vSonarRight = vrx[4]
        vSonarFrontLeft = vrx[5]
        vSonarFrontRight = vrx[6]

        vEncoderFrontLeft = vrx[7]
        vEncoderFrontRight = vrx[8] 
        vEncoderRearLeft = vrx[9]
        vEncoderRearRight = vrx[10]
        
        vHeading = vrx[11]
        
        vXRob = vrx[12]
        vYRob = vrx[13]
        vZRob = vrx[14]
        vXAcc = vrx[15]
        vYAcc = vrx[16]
        vZAcc = vrx[17]
        vXGyro = vrx[18]
        vYGyro = vrx[19]
        vZGyro = vrx[20]

        vEncoderRearLeftReset = vrx[21]
        vEncoderRearRightReset = vrx[22]

        #print ("update params in vSimVar from vrep values ...")
        self.vSimTime = simulationTime
        self.__vSimVar["vLocation"] = [vXRob,vYRob,vZRob]
        self.__vSimVar["vAccelX"] = int(round(vXAcc*1000.0))
        self.__vSimVar["vAccelY"] = int(round(vYAcc*1000.0))
        self.__vSimVar["vAccelZ"] = int(round(vZAcc*1000.0))
        self.__vSimVar["vGyroX"] = int(round(vXGyro*1000.0))
        self.__vSimVar["vGyroY"] = int(round(vYGyro*1000.0))
        self.__vSimVar["vGyroZ"] = int(round(vZGyro*1000.0))
        self.__vSimVar["vMagX"] = int(round(math.cos(vHeading*math.pi/180.0)*1000.0))
        self.__vSimVar["vMagY"] = int(round(math.sin(vHeading*math.pi/180.0)*1000.0))
        self.__vSimVar["vMagZ"] = int(round(1000.0))
        self.__vSimVar["vHeading"] = vHeading
        self.__vSimVar["vEncoderFrontLeft"] = self.actual_front_encoders(vEncoderFrontLeft)
        self.__vSimVar["vEncoderFrontRight"] = self.actual_front_encoders(vEncoderFrontRight)
        self.__vSimVar["vEncoderRearLeft"] =  self.actual_rear_encoders(vEncoderRearLeft)
        self.__vSimVar["vEncoderRearRight"] =  self.actual_rear_encoders(vEncoderRearRight)
        if vEncoderRearLeftReset == -1.0:
            self.__vSimVar["vEncoderRearLeftReset"] = 0.0
            #print ("-- reset left")
        if vEncoderRearRightReset == -1.0:
            self.__vSimVar["vEncoderRearRightReset"] = 0.0
            #print ("-- reset right")
        self.__vSimVar["vSonarFrontLeft"] = self.actual_sonar(vSonarFrontLeft)
        self.__vSimVar["vSonarFrontRight"] = self.actual_sonar(vSonarFrontRight)
        self.__vSimVar["vSonarLeft"] = self.actual_sonar(vSonarLeft)
        self.__vSimVar["vSonarRight"] = self.actual_sonar(vSonarRight)
        self.__vSimVar["vSonarFront"] = self.actual_sonar(vSonarFront)
        self.__vSimVar["vSonarRear"] = self.actual_sonar(vSonarRear)
        #self.__trex.status["left_encoder"] =  self.actual_front_encoders(vEncoderFrontLeft)
        #self.__trex.status["right_encoder"] = self.actual_front_encoders(vEncoderFrontRight)

    # battery level v->bin
    def battery_voltage_v2bin_old(self,v):
        vb = int(round((1024*0.23*v)/5.0))
        return vb
    def battery_voltage_v2bin(self,v):
        vb = int(round((1024*v)/(5.0*4.3)))
        return vb
        
    # actual sonar functions
    def actual_sonar(self,v):
        v = round(v*100)
        if random.random() < 0.005: # i2c failure probability of 0.5 %
            v = -1
        return v

    # actual encoder functions
    def actual_front_encoders(self,v): # 16 bits signed
        iv = int(round(v+0.5)) % 65536  # put on  16 bits
        iv0 = iv
        if iv > 32767:   # add the sign
            iv -= 65536
        return iv
    
    def actual_rear_encoders(self,v): # 16 bits unsigned
        iv = int(round(v+0.5)) % 65536  # put on  16 bits
        while iv < 0:
            iv += 65536
        #if random.random() < 0.005: # i2c failure probability of 0.5 %
        #    iv = -1
        return iv

    # clean stop of simulation
    def end(self):
        if self.__dartSim:        
            #print (self.sonars.vsv)
            for isn in range(6):
                sonar_num = isn+1
                kySimEnd = "vSonar%1dSimEnd"%(sonar_num)
                #print ("stop sonar",sonar_num)
                self.sonars.vsv[kySimEnd] = True   
            #print (self.sonars.vsv)
            for isn in range(6):
                sonar_num = isn+1
                kySim = "vSonar%1dSim"%(sonar_num)
                while True:
                    #print ("check for end, sonar",sonar_num)
                    if not self.sonars.vsv[kySim]:
                        break
                    #print ("wait for end, sonar",sonar_num)
                    time.sleep(1.0)
                    pass
                #print ("sonar",sonar_num," ended ...")
            self.__simulation_alive = False
            #print (self.__simulation_alive)
            time.sleep(0.1)
            print ("... end")

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
        if self.__dartSim:
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
        if self.__dartSim:
            encFrontLeft = self.__vSimVar["vEncoderFrontLeft"]
            encFrontRight = self.__vSimVar["vEncoderFrontRight"]
        else:
            status = self.__trex.status
            #print ("status",status)
            encFrontLeft = status["left_encoder"]
            encFrontRight = status["right_encoder"]
        return [encFrontLeft,encFrontRight]

    def update_front_encoders(self,t,odofl,odofr):
        # refresh encoders current and memory values    
        self.encoders_front_acq_time = t
        self.encoders_front_left_mem  = self.encoders_front_left_last
        self.encoders_front_right_mem  = self.encoders_front_right_last
        self.encoders_front_left_last = odofl
        self.encoders_front_right_last = odofr
        
    def get_rear_encoders(self):
        if self.__dartSim:
            encRearLeft = self.__vSimVar["vEncoderRearLeft"]
            encRearRight = self.__vSimVar["vEncoderRearRight"]
            #print ("encRearLeft,encRearRight",encRearLeft,encRearRight)
        else:
            encRearLeft,encRearRight = self.encoders.read_encoders_both()
        return [encRearLeft,encRearRight]

    def update_rear_encoders(self,t,odorl,odorr):
        # refresh encoders current and memory values    
        self.encoders_rear_acq_time = t
        self.encoders_rear_left_mem  = self.encoders_rear_left_last
        self.encoders_rear_right_mem  = self.encoders_rear_right_last
        self.encoders_rear_left_last = odorl
        self.encoders_rear_right_last = odorr
    
    def update_cardinal_sonars(self,t,df,dl,db,dr):
        self.sonars_cardinal_acq_time = t
        self.sonars_cardinal_front = df
        self.sonars_cardinal_left = dl
        self.sonars_cardinal_rear = db
        self.sonars_cardinal_right = dr

    def update_diagonal_sonars(self,t,dl,dr):
        self.sonars_diagonal_acq_time = t
        self.sonars_diagonal_left = dl
        self.sonars_diagonal_right = dr
          

if __name__ == "__main__":
    print ("start")
    myDart = DartV2Basis()
    print (dir(MyDart))

    time.sleep(0.05)
    myDart.end()
    



