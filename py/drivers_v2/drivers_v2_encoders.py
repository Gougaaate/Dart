#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import sys
import time

class EncodersIO():
    def __init__(self,exec_robot,vsv=None):
        self.vsv = vsv
        self.__exec_robot = exec_robot
        self.__sim = False
        self.__ros = False
        self.__real = False
        if exec_robot == "Sim V-REP":
            self.__sim = True
        elif exec_robot == "Sim GAZEBO":
            self.__sim = True
        elif exec_robot == "Real":
            self.__real = True
        elif exec_robot == "Real ROS":
            self.__ros = True

        self.__bus_nb = 2
        self.__addr = 0x14 

        # conditional i2c setup
        # if real robot , then we use actual i2c
        # if not , we are on simulated i2c
        if self.__sim:
            import i2csim as i2c
            self.__dev_i2c=i2c.i2c(self.__addr,self.__bus_nb,vsv=self.vsv)
        elif self.__real:
            import i2creal as i2c
            self.__dev_i2c=i2c.i2c(self.__addr,self.__bus_nb)
        elif self.__ros:
            import dartv2_drivers.drivers.i2creal as i2c
            self.__dev_i2c=i2c.i2c(self.__addr,self.__bus_nb)
            
    def get_version(self):
        return self.__read_byte(0xC0)

    def reset_left(self):
        return self.__write_byte(0x02,0)
    
    def reset_right(self):
        return self.__write_byte(0x01,0)

    def reset_both(self):
        return self.__write_byte(0x05,0)

    def battery_voltage(self):
        #time.sleep(0.001)
        offs = 6
        v = self.__read(offs)
        #print ("batlvl",v)
        vmes = 5.0*v/1024.0
        vbat = vmes * 430.0/100.0
        #print (v,vmes,vbat)
        return vbat

    def read_encoders_both_byte_test (self):
        rl = self.__read_byte(0x00)
        rh = self.__read_byte(0x01)
        ll = self.__read_byte(0x02)
        lh = self.__read_byte(0x03)
        

    def read_encoders(self):
        offs = 0
        self.enc_left = self.__read(offs)
        offs = 2
        self.enc_right = self.__read(offs)
        return [self.enc_left, self.enc_right]

    def read_encoders_both (self):
        offs = 0
        i2c_ok = True
        try:
            v = self.__dev_i2c.read(offs,4)
        except:
            i2c_ok = False
        if i2c_ok:
            self.enc_left =  v[0] + (v[1] << 8)
            self.enc_right =  v[2] + (v[3] << 8)
        return [self.enc_left, self.enc_right]

    def read_motors_direction (self):
        time.sleep(0.001)
        offs = 4
        self.motor_dir_left = self.__read_byte(offs)
        time.sleep(0.001)
        offs = 5
        self.motor_dir_right = self.__read_byte(offs)
        return [self.motor_dir_left, self.motor_dir_right]
              
    def __read(self,offs):
        v=0
        while True:
            try:
                v = self.__dev_i2c.read(offs,2)
                #print (v)
                v = v[0] + (v[1] << 8)
                break
            except:
                v = None
                time.sleep(0.0005)
        return v        

    def __read_byte (self,offs):
        v=0
        try:
            v = self.__dev_i2c.read_byte(offs)
        except:
            v = None
        return v
    
    def __write_byte (self,offs,val):
        v = 0
        try:
            self.__dev_i2c.write(offs,[val])
        except:
            v = None
        return v

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
        encoders = EncodersIO("Sim V-REP",vsv=tSimVar)
        # initiate communication thread with V-Rep
        tSimVar["vSimAlive"] = False
        import vrep_interface as vrep
        vrep_itf = vrep.VrepInterface(tSimVar)
        vrep = vrep_itf.start_thread()
        print ("vDart ready ...")
        print ("Simulation alive is ",tSimVar["vSimAlive"])
        print ("Work with virtual DART on V-REP")


    print ("Encoder Microcode Version : ",encoders.get_version())
    print ("Battery Voltage (V) :",encoders.battery_voltage())
    print ("Encoders :",encoders.read_encoders())
    print ("Motors direction :",encoders.read_motors_direction())
    print ("Reset Encoders")
    encoders.reset_both()
    encoders.vsv["vSimAlive"] = False