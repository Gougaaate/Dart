#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import sys
import os
import math

class Imu9IO():
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
        self.__addr_mg = 0x1e  # mag sensor
        self.__addr_ag = 0x6b  # accelero - gyro

        # conditional i2c setup
        # if real robot , then we use actual i2c
        # if not , we are on simulated i2c
        if self.__sim:
            import i2csim as i2c
            self.__dev_i2c_mg=i2c.i2c(self.__addr_mg,self.__bus_nb,vsv=self.vsv)
            self.__dev_i2c_ag=i2c.i2c(self.__addr_ag,self.__bus_nb,vsv=self.vsv)
        elif self.__real:
            import i2creal as i2c
            self.__dev_i2c_mg=i2c.i2c(self.__addr_mg,self.__bus_nb)
            self.__dev_i2c_ag=i2c.i2c(self.__addr_ag,self.__bus_nb)
        elif self.__ros:
            import dartv2_drivers.drivers.i2creal as i2c
            self.__dev_i2c_mg=i2c.i2c(self.__addr_mg,self.__bus_nb)
            self.__dev_i2c_ag=i2c.i2c(self.__addr_ag,self.__bus_nb)
            
        self.magx_min = 0.0
        self.magx_max = 0.0
        self.magy_min = 0.0
        self.magy_max = 0.0
        self.magx_offs = 0.0
        self.magy_offs = 0.0
        self.magx_scale = 0.0
        self.magy_scale = 0.0

        self.__mag_raw = [0.0,0.0,0.0]
        self.__accel_raw = [0.0,0.0,0.0]
        self.__gyro_raw = [0.0,0.0,0.0]

        if self.__sim:
            # device non implemented in I2C simulation
            pass
        else:
            # configure mag sensor
            # CTRL_REG1 (0x20) = 0b01110000
            # OM = 11 (ultra-high-performance mode for X and Y);
            # DO = 100 (10 Hz ODR)
            self.__dev_i2c_mg.write(0x20,[0x70])
            # CTRL_REG2 (0x21) = 0b00000000
            # FS = 00 (+/- 4 gauss full scale)
            self.__dev_i2c_mg.write(0x21,[0x00])
            # CTRL_REG3 (0x22) = 0b00000000
            # MD = 00 (continuous-conversion mode)
            self.__dev_i2c_mg.write(0x22,[0x00])
            # CTRL_REG4 (0x23) = 0b00001100
            # OMZ = 11 (ultra-high-performance mode for Z)
            self.__dev_i2c_mg.write(0x23,[0x0C])
            #
            # configure accelero + gyro
            # LSM6DS33 gyro
            # CTRL2_G (0x11) = 0b10001100
            # ODR = 1000 (1.66 kHz (high performance))
            # FS_G = 11 (2000 dps)
            self.__dev_i2c_ag.write(0x11,[0x8C])
            #
            # CTRL7_G (0x16) = 0b00000000
            # defaults
            self.__dev_i2c_ag.write(0x16,[0x00])
            #
            # LSM6DS33 accelerometer
            # CTRL1_XL (0x10) = 0b10001100
            # ODR = 1000 (1.66 kHz (high performance))
            # FS_XL = 11 (8 g full scale)
            # BW_XL = 00 (400 Hz filter bandwidth)
            #self.__dev_i2c_ag.write(0x10,[0x8C])
            # more filtering BW_XL = 11 (50 Hz filter bandwidth)
            self.__dev_i2c_ag.write(0x13,[0x00])
            self.__dev_i2c_ag.write(0x10,[0x8C])
            #
            # common
            # CTRL3_C (0x12) 0b00000100
            # IF_INC = 1 (automatically increment address register)
            self.__dev_i2c_ag.write(0x12,[0x04])

    def setup_accel_filter (self,mode):
        if mode == 0:
            self.__dev_i2c_ag.write(0x17,[0x00])
            self.__dev_i2c_ag.write(0x13,[0x00])
            self.__dev_i2c_ag.write(0x10,[0x8C])
        elif mode == 1:
            self.__dev_i2c_ag.write(0x17,[0x00])
            self.__dev_i2c_ag.write(0x13,[0x80])
            self.__dev_i2c_ag.write(0x10,[0x8F])
        elif mode == 2:
            self.__dev_i2c_ag.write(0x17,[0x80])
            self.__dev_i2c_ag.write(0x13,[0x80])
            self.__dev_i2c_ag.write(0x10,[0x8F])       
        
    def read_mag_raw(self):
        v = self.__dev_i2c_mg.read(0x28,6)
        ix = self.cmpl2(v[0],v[1])
        iy = self.cmpl2(v[2],v[3])
        iz = self.cmpl2(v[4],v[5])
        self.__mag_raw = [ix,iy,iz]
        return self.__mag_raw
 
    def read_gyro_raw(self):
        # OUTX_L_G (0x22)
        v = self.__dev_i2c_ag.read(0x22,6)
        ix = self.cmpl2(v[0],v[1])
        iy = self.cmpl2(v[2],v[3])
        iz = self.cmpl2(v[4],v[5])
        self.__gyro_raw = [ix,iy,iz]
        return self.__gyro_raw
 
    def read_accel_raw(self):
        # OUTX_L_XL (0x28)
        v = self.__dev_i2c_ag.read(0x28,6)
        ix = self.cmpl2(v[0],v[1])
        iy = self.cmpl2(v[2],v[3])
        iz = self.cmpl2(v[4],v[5])
        self.__accel_raw = [ix,iy,iz]
        return self.__accel_raw

    def cmpl2(self,lsByte,msByte):
        i = lsByte + (msByte << 8)
        if i >= (1<<15):
            i = i - (1<<16)
        return i

    def heading_raw(self,magx=1,magy=0):
        if self.__sim:
            heading = self.vsv["vHeading"]*math.pi/180.0
        else:
            heading = math.atan2(magy,magx)
        return heading
        
    def heading_raw_deg(self,magx=1,magy=0):
        if self.__sim:
            heading = self.vsv["vHeading"]
        else:
            heading = self.heading_raw(magx,magy)*180.0/math.pi
            if heading < 0.0:
                heading += 360.0
        return heading

    def fast_heading_calibration (self, magx_min, magx_max, magy_min, magy_max):
        if self.__sim:
            pass
        else:
            self.magx_min = magx_min
            self.magx_max = magx_max
            self.magy_min = magy_min
            self.magy_max = magy_max
            self.magx_offs = (magx_min+magx_max)/2.0
            self.magy_offs = (magy_min+magy_max)/2.0
            self.magx_scale = 2./(magx_max-magx_min)
            self.magy_scale = 2./(magy_max-magy_min)
        
    def heading(self,magx=1,magy=0):
        if self.__sim:
            heading = self.vsv["vHeading"]*math.pi/180.0
        else:
            magx_cal = (magx - self.magx_offs) * self.magx_scale
            magy_cal = (magy - self.magy_offs) * self.magy_scale
            #print (self.magx_min,magx,self.magx_max,self.magy_min,magy,self.magy_max,magx_cal,magy_cal)
            magx_cal = 1.0 if magx_cal>1.0 else magx_cal
            magx_cal = -1.0 if magx_cal<-1.0 else magx_cal
            magy_cal = 1.0 if magy_cal>1.0 else magy_cal
            magy_cal = -1.0 if magy_cal<-1.0 else magy_cal
            heading = math.atan2(magy_cal,magx_cal)
        return heading
        
    def heading_deg(self,magx=1,magy=0):
        if self.__sim:
            heading = self.vsv["vHeading"]
        else:
            heading = self.heading(magx,magy)*180.0/math.pi
            if heading < 0.0:
                heading += 360.0
        return heading

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
        imu = Imu9IO("Sim V-REP",vsv=tSimVar)
        # initiate communication thread with V-Rep
        tSimVar["vSimAlive"] = False
        import vrep_interface as vrep
        vrep_itf = vrep.VrepInterface(tSimVar)
        vrep = vrep_itf.start_thread()
        print ("vDart ready ...")
        print ("Simulation alive is ",tSimVar["vSimAlive"])
        print ("Work with virtual DART on V-REP")

    print ("heading raw (rad)",imu.heading_raw())
    print ("heading raw (deg)",imu.heading_raw_deg())
    print ("heading (rad)",imu.heading())
    print ("heading (deg)",imu.heading_deg())
    if tstsim:
        time.sleep(0.25)
        tSimVar["vSimAlive"] = False
        time.sleep(0.25)
