#!/usr/bin/python
# -*- coding: utf-8 -*-
import threading
import socket
import time
import math
import random

class VrepInterface():
    def __init__(self,tSimVar,debug=False,port=30100):
        self.__tSimVar = tSimVar 
        self.__debug = debug
        self.dtmx = -1.0
        self.dtmn = 1e38
        self.cnt_sock = 0
        self.dta_sock = 0.0
        self.upd_sock = False
        self.__tSimVar["vSimAlive"] = False
        self.vdart_ready = None
        self.__HOST = 'localhost'  # IP of the sockect
        self.__PORT = port # port (set similarly in v-rep, default 30100)
        self.server_address = (self.__HOST, self.__PORT)
        self.vrep = None

    def start_thread(self):
        self.vdart_ready = threading.Event()
        self.vdart_ready.clear()
        # socket connection to V-REP
        srv = self.server_address
        ev = self.vdart_ready
        self.__tSimVar["vSimAlive"] = True
        self.vrep = threading.Thread(target=self.vrep_com_socket,args=(srv,ev,))
        self.vrep.start()
        # wait for vdart to be ready
        self.vdart_ready.wait()


    def vrep_com_socket(self,srv,ev):
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
            vDoLog = float(self.__tSimVar["vDoLog"])
            vCmdSpeedLeft = self.__tSimVar["vCmdSpeedLeft"]
            vCmdSpeedRight = self.__tSimVar["vCmdSpeedRight"]
            vCmdSpeedNew = self.__tSimVar["vCmdSpeedNew"]
            vEncoderRearLeftReset = self.__tSimVar["vEncoderRearLeftReset"]
            vEncoderRearRightReset = self.__tSimVar["vEncoderRearRightReset"]
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

            #print ("rx data len: ",len(data))
            rxHeader = struct.unpack('<ccHH',data[0:6])
            rxDataSize = 6 + rxHeader[2] + 256*rxHeader[3]
            while True:
                data_nw = sock.recv(200)
                if len(data_nw) == 0:
                    #print (len(data_nw))
                    break
                else:
                    data += data_nw
                    #print (len(data))
            #print (rxHeader,rxDataSize,len(data))
            if len(data) >= rxDataSize:
                vrx = struct.unpack('<ccHHfffffffffffffffffffffff',data[0:rxDataSize])
                self.vrep_update_sim_param(upd_sock,vrx[4:])
            else:
                print ("bad data length ",len(data))

            sock.close()
            self.cnt_sock = self.cnt_sock + 1
            tsock = (time.time() - t0)*1000.0
            self.dta_sock += tsock
            if tsock > self.dtmx:
                self.dtmx = tsock
            if tsock < self.dtmn:
                self.dtmn = tsock
            dtm = self.dta_sock/float(self.cnt_sock)
            #print ("tsock",tsock)
            if self.__debug:
                if (self.cnt_sock % 100) == 99:
                    print ("min,mean,max socket thread duration (ms) : ",self.dtmn,dtm,self.dtmx)

            #time.sleep(0.2)
            #print (dir(ev))
            
            if vCmdSpeedNew != 0.0:
                self.__tSimVar["vCmdSpeedNew"] = 0.0 # reset motor change cmd 
            ev.set()

            #print (self.__simulation_alive)
            if not self.__tSimVar["vSimAlive"]:
                break 
            t1 = time.time()
            tsleep = 0.009 - (t1-t0)
            if tsleep > 0:
                time.sleep(tsleep)
            

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
        
        vHeading = vrx[11] # trigonometric heading
        # compute geographic heading so that North is 0
        vHeading = vHeading + 180
        # limit heading to [0, 360] degrees
        while vHeading > 360.0:
            vHeading -= 360.0
        while vHeading < 0.0:
            vHeading += 360.0
        
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

        #print ("update params in tSimVar from vrep values ...")
        self.vSimTime = simulationTime
        self.__tSimVar["vLocation"] = [vXRob,vYRob,vZRob]
        accelCoef = 10.0
        self.__tSimVar["vAccelX"] = int(round(vXAcc*accelCoef))
        self.__tSimVar["vAccelY"] = int(round(vYAcc*accelCoef))
        self.__tSimVar["vAccelZ"] = int(round(vZAcc*accelCoef))
        gyroCoef = 10.0
        self.__tSimVar["vGyroX"] = int(round(vXGyro*gyroCoef))
        self.__tSimVar["vGyroY"] = int(round(vYGyro*gyroCoef))
        self.__tSimVar["vGyroZ"] = int(round(vZGyro*gyroCoef))
        magCoef = 1000.0
        self.__tSimVar["vMagX"] = int(round(math.cos(vHeading*math.pi/180.0)*magCoef))
        self.__tSimVar["vMagY"] = int(round(math.sin(vHeading*math.pi/180.0)*magCoef))
        self.__tSimVar["vMagZ"] = int(round(magCoef))
        self.__tSimVar["vHeading"] = vHeading
        self.__tSimVar["vEncoderFrontLeft"] = self.actual_front_encoders(vEncoderFrontLeft)
        self.__tSimVar["vEncoderFrontRight"] = self.actual_front_encoders(vEncoderFrontRight)
        self.__tSimVar["vEncoderRearLeft"] =  self.actual_rear_encoders(vEncoderRearLeft)
        self.__tSimVar["vEncoderRearRight"] =  self.actual_rear_encoders(vEncoderRearRight)
        if vEncoderRearLeftReset == -1.0:
            self.__tSimVar["vEncoderRearLeftReset"] = 0.0
            #print ("-- reset left")
        if vEncoderRearRightReset == -1.0:
            self.__tSimVar["vEncoderRearRightReset"] = 0.0
            #print ("-- reset right")
        self.__tSimVar["vSonarFrontLeft"] = self.actual_sonar(vSonarFrontLeft)
        self.__tSimVar["vSonarFrontRight"] = self.actual_sonar(vSonarFrontRight)
        self.__tSimVar["vSonarLeft"] = self.actual_sonar(vSonarLeft)
        self.__tSimVar["vSonarRight"] = self.actual_sonar(vSonarRight)
        self.__tSimVar["vSonarFront"] = self.actual_sonar(vSonarFront)
        self.__tSimVar["vSonarRear"] = self.actual_sonar(vSonarRear)
        #self.__trex.status["left_encoder"] =  self.actual_front_encoders(vEncoderFrontLeft)
        #self.__trex.status["right_encoder"] = self.actual_front_encoders(vEncoderFrontRight)
        self.__tSimVar["vVoltageBin"] = self.battery_voltage_v2bin(self.__tSimVar["vVoltage"])

    # sensor functions to make them more close to reel robot
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

    # battery level v->bin
    def battery_voltage_v2bin_old(self,v):
        vb = int(round((1024*0.23*v)/5.0))
        return vb
    def battery_voltage_v2bin(self,v):
        vb = int(round((1024*v)/(5.0*4.3)))
        return vb

