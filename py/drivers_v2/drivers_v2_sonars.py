#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
import time
import threading

class SonarsIO():
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
        self.__addr_4_sonars = 0x21
        self.__addr_front_left = 0x070
        self.__addr_front_right = 0x072

        # conditional i2c setup
        # if real robot , then we use actual i2c
        # if not , we are on simulated i2c
        if self.__sim:
            import i2csim as i2c
            self.__dev_i2c_4_sonars=i2c.i2c(self.__addr_4_sonars,bus_nb=self.__bus_nb,vsv=self.vsv)
            self.__dev_i2c_front_left=i2c.i2c(self.__addr_front_left,bus_nb=self.__bus_nb,vsv=self.vsv)
            self.__dev_i2c_front_right=i2c.i2c(self.__addr_front_right,bus_nb=self.__bus_nb,vsv=self.vsv)
        elif self.__real:
            import i2creal as i2c
            self.__dev_i2c_4_sonars=i2c.i2c(self.__addr_4_sonars,self.__bus_nb)
            self.__dev_i2c_front_left=i2c.i2c(self.__addr_front_left,self.__bus_nb)
            self.__dev_i2c_front_right=i2c.i2c(self.__addr_front_right,self.__bus_nb)
        elif self.__ros:
            import dartv2_drivers.drivers.i2creal as i2c
            self.__dev_i2c_4_sonars=i2c.i2c(self.__addr_4_sonars,self.__bus_nb)
            self.__dev_i2c_front_left=i2c.i2c(self.__addr_front_left,self.__bus_nb)
            self.__dev_i2c_front_right=i2c.i2c(self.__addr_front_right,self.__bus_nb)            

        for isn in range(4):
            self.set_mode(isn+1,2)        
        self.front = -1.0
        self.left = -1.0
        self.right = -1.0
        self.rear = -1.0
        self.diag_left = -1.0
        self.diag_right = -1.0
        self.diag_left_w = -1.0
        self.diag_right_w = -1.0      

    def get_version(self):
        return self.__dev_i2c_4_sonars.read_byte(0xC0)

    def get_mode(self,num):
        return self.__dev_i2c_4_sonars.read_byte(0xB0+num-1)

    def get_state(self,num):
        return self.__dev_i2c_4_sonars.read_byte(0xB4+num-1)

    def set_mode(self,num,mode):
        vals = [mode]
        self.__dev_i2c_4_sonars.write(0xB0+num-1,vals)

    def set_dist_max(self,num,dist):
        offs = 0xA0+(num-1)*2
        idist = int(round(dist*100.0))
        vals = [idist%256,idist//256]
        self.__dev_i2c_4_sonars.write(offs,vals)

    def init_4_sonars (self,dmax=1.5,mode="sync"):
        # set dmax for 4 sonars in mode 1 and for mode sync
        for isn in range(5):
            self.set_dist_max(isn+1,dmax)
        if mode == "sync":
            mode = 2
            for isn in range(4):
                self.set_mode(isn+1,mode)
        
    def read_4_sonars(self,unit="cm"):
        vf = self.read_front()
        vl = self.read_left()
        vb = self.read_rear()
        vr = self.read_right()
        if unit == "cm":
            return [vf, vl, vb, vr]
        elif unit == "m":
            return [vf/100.0, vl/100.0, vb/100.0, vr/100.0]

    def read_front(self):
        self.front = self.__read(1)
        return self.front
    
    def read_front_bytes (self):
        addr = 0x00
        vl = self.__dev_i2c_4_sonars.read_byte(addr)
        addr = 0x01
        vh = self.__dev_i2c_4_sonars.read_byte(addr)
        self.front = vl+vh*256 
        return self.front

    def read_left(self):
        self.left = self.__read(3)
        return self.left

    def read_rear(self):
        self.rear = self.__read(2)
        return self.rear
    
    def read_right(self):
        self.right = self.__read(4)
        return self.right

    def __read(self,num_sonar):
        try:
            addr = 0x00+2*(num_sonar-1)
            #print ("addr",addr)
            v = self.__dev_i2c_4_sonars.read(addr,2)
            #print ("v",v)
            v = v[0] + (v[1] << 8)
            #print ("v",v)
        except:
            print ("------ error I2C sonar")
            v = -1
        return v

    def __write(self,cmd):
        #self.bus.write_i2c_block_data(self.addr,0,cmd)
        self.__dev_i2c_4_sonars.write(0,cmd)


    # read two data bytes at once
    def read_diag_left_word(self):
        try:
            self.__dev_i2c_front_left.write(0,[0x51])
            time.sleep(0.065)
            try:
                vms,vls = self.__dev_i2c_front_left.read(2,2)
                #print (vms,vls)
                self.diag_left_w = float(vls + (vms << 8))/100.0
            except:
                self.diag_left_w = -1
        except:
            self.diag_left_w = -1
        return self.diag_left_w

    # read two data bytes at once
    def read_diag_right_word(self):
        try:
            self.__dev_i2c_front_right.write(0,[0x51])
            time.sleep(0.065)
            try:
                vms,vls = self.__dev_i2c_front_right.read(2,2)
                #print (vms,vls)
                self.diag_right_w = float(vls + (vms << 8))/100.0
            except:
                self.diag_right_w = -1
        except:
            self.diag_right_w = -1
        return self.diag_right_w

    # read two bytes separately
    def read_diag_left(self):
        #self.bus.write_i2c_block_data(self.addr_l,0,[0x51])
        try:
            self.__dev_i2c_front_left.write(0,[0x51])
            time.sleep(0.065)
            try:
                vms = self.__dev_i2c_front_left.read_byte(2)
                vls = self.__dev_i2c_front_left.read_byte(3)
                #print (vms,vls)
                self.diag_left = float(vls + (vms << 8))/100.0
            except:
                self.diag_left = -1
        except:
            self.diag_left = -1
        return self.diag_left

    # read two bytes separately
    def read_diag_right(self):
        #self.bus.write_i2c_block_data(self.addr_r,0,[0x51])
        try:
            self.__dev_i2c_front_right.write(0,[0x51])
            time.sleep(0.065)
            try:
                vms = self.__dev_i2c_front_right.read_byte(2)
                vls = self.__dev_i2c_front_right.read_byte(3)
                #print (vms,vls)
                self.diag_right = float(vls + (vms << 8))/100.0        
            except:
                self.diag_right = -1
        except:
            self.diag_right = -1
        return self.diag_right

    def read_diag_all(self):
        self.read_diag_both()
        return [self.diag_left,self.diag_right]

    def read_diag_both(self):
        #self.bus.write_i2c_block_data(self.addr_l,0,[0x51])
        #self.bus.write_i2c_block_data(self.addr_r,0,[0x51])
        try:
            self.__dev_i2c_front_left.write(0,[0x51])
            self.__dev_i2c_front_right.write(0,[0x51])
            time.sleep(0.065)
            try:
                vms = self.__dev_i2c_front_left.read_byte(2)
                vls = self.__dev_i2c_front_left.read_byte(3)
                #vms = self.bus.read_byte_data(self.addr_l,2)
                #vls = self.bus.read_byte_data(self.addr_l,3)
                #print (vms,vls)
                self.diag_left = float(vls + (vms << 8))/100.0
            except:
                self.diag_left = -1
            try:
                vms = self.__dev_i2c_front_right.read_byte(2)
                vls = self.__dev_i2c_front_right.read_byte(3)
                #vms = self.bus.read_byte_data(self.addr_r,2)
                #vls = self.bus.read_byte_data(self.addr_r,3)
                #print (vms,vls)
                self.diag_right = float(vls + (vms << 8))/100.0        
            except:
                self.diag_right = -1
        except:
            self.diag_left = -1
            self.diag_right = -1
        return [self.diag_left,self.diag_right]

        
    def get_distance(self, sonar_key):
        """
        Return distance measured "sonar_key" sonar in meters
        
        Parameters:
            sonar_key: string, one of "front", "rear", "left", "right"
            
        Return value:
            distance: float, -1 on error, 0 on timeOut
        """
        v = -1
        n = 0
        if sonar_key == "front":
            v = self.read_front()
            n = 1
        elif sonar_key == "rear":
            v = self.read_rear()
            n = 1
        elif sonar_key == "left":
            v = self.read_left()
            n = 1
        elif sonar_key == "right":
            v = self.read_right()
            n = 1
        elif sonar_key == "front_left":
            v = self.read_diag_left()*100.0
            n = 1
        elif sonar_key == "front_right":
            v = self.read_diag_right()*100.0
            n = 1
        elif sonar_key == "front_diag":
            v = self.read_diag_all()
            n = 2
        if n==1:
            if v>0:
                v = float(v)/100.0
        return v

if __name__ == "__main__":
    # warning, tests are quite complex in simulation as we need to connect
    # the module to the V-REP simulator...

    # test if on real robot , if gpio exists (not very robust)
    # add test on processor type, real robot has armv7l
    tstsim = False
    if (os.access("/sys/class/gpio/gpio266", os.F_OK)) \
       and (platform.processor() == 'armv7l'):
        sonars = SonarsIO("Real")
        print ("Work with real DART")
    # if not the virtual robot is running in V-REP
    else :
        tstsim = True
        sys.path.append('../vDartV2')
        import vSimVar as vsv
        tSimVar= vsv.tSimVar
        sonars = SonarsIO("Sim V-REP",vsv=tSimVar)
        # initiate communication thread with V-Rep
        tSimVar["vSimAlive"] = False
        import vrep_interface as vrep
        vrep_itf = vrep.VrepInterface(tSimVar)
        vrep = vrep_itf.start_thread()
        print ("vDart ready ...")
        print ("Simulation alive is ",tSimVar["vSimAlive"])
        print ("Work with virtual DART on V-REP")

    print ("Version : ",sonars.get_version())

    mode = 2
    dmax = 1.5
    for isn in range(4):
        #print ("Mode",isn+1,":","%2.2X"%(sonars.get_mode(isn+1)))
        sonars.set_mode(isn+1,mode)
        sonars.set_dist_max(isn+1,2.0)
    sonars.set_dist_max(5,dmax)
    for isn in range(4):
        #print ("Mode",isn+1,":","%2.2X"%(sonars.get_mode(isn+1)))
        pass
    for isn in range(4):
        print ("State",isn+1,":","%2.2X"%(sonars.get_state(isn+1)))
    for itst in range(5):
        time.sleep(0.5)
        print ("Sonars cardinal",sonars.read_4_sonars())
        print ("Sonars front diag",sonars.read_diag_both())
    #print (dir(sonars))
    if tstsim:
        #print (sonars.vsv)
        for isn in range(6):
            sonar_num = isn+1
            kySimEnd = "vSonar%1dSimEnd"%(sonar_num)
            print ("sim: stop sonar",sonar_num)
            sonars.vsv[kySimEnd] = True   
        #print (sonars.vsv)
        for isn in range(6):
            sonar_num = isn+1
            kySim = "vSonar%1dSim"%(sonar_num)
            while True:
                print ("sim: check for end, sonar",sonar_num)
                if not sonars.vsv[kySim]:
                    break
                print ("sim: wait for end, sonar",sonar_num)
                time.sleep(1.0)
                pass
            print ("sim: sonar",sonar_num," ended ...")
        sonars.vsv["vSimAlive"] = False

    for thread in threading.enumerate(): 
        print ("thread still alive :",thread.name,thread.is_alive())
