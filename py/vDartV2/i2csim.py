"""
0x07 : T-Rex motor driver (not managed by virtual I2C driver)
0x14 : Rear encoders and battery level
0x1e : First address of Pololu IMU 9
0x21 : 4 cardinal sonars
0x40 : 7 segments display
0x6b : Second address of Pololu IMU 9
0x70 : Front left diagonal sonar (on 7 segm display side)
0x72 : Front right diagonal sonar
"""
#import imp
#i2c = imp.load_source('i2c', './vI2C.py')
#print (dir(i2c)) 

import threading
import random
import time

class i2c():
    # same functions as real I2C
    def __init__(self,addr,bus_nb=2,vsv=None):
        self.__bus_nb = bus_nb
        self.__addr = addr
        #self.__bus = smbus.SMBus(self.__bus_nb)
        print ("init i2c device addr = 0x%x on bus %d"%(addr,bus_nb))
        #
        # I2C sim fun
        self.__i2cCodes = self.i2cSimCodes()
        self.vsv = vsv
        # print ("vsv (i2c)",vsv,self.vsv)
        # Start Sonar Simulator in separate threads
        # 4 cardinal sonars
        self.__sim_sonar_thread = [None,None,None,None,None,None]
        if addr==0x21:
            for isn in range(4):
                sonar_num = isn+1
                ky = "vSonar%1dSim"%(sonar_num)
                if not vsv[ky]:
                    vsv[ky] = True
                    self.__sim_sonar_thread[isn] = threading.Thread(target=self.sim_sonar,args=(sonar_num,self.vsv,))
                    self.__sim_sonar_thread[isn].start()
        elif addr==0x70:
            sonar_num = 5
            ky = "vSonar%1dSim"%(sonar_num)
            if not vsv[ky]:
                vsv[ky] = True
                self.__sim_sonar_thread[sonar_num-1] = threading.Thread(target=self.sim_sonar_diag,args=(sonar_num,self.vsv,))
                self.__sim_sonar_thread[sonar_num-1].start()
        elif addr==0x72:
            sonar_num = 6
            ky = "vSonar%1dSim"%(sonar_num)
            if not vsv[ky]:
                vsv[ky] = True
                self.__sim_sonar_thread[sonar_num-1] = threading.Thread(target=self.sim_sonar_diag,args=(sonar_num,self.vsv,))
                self.__sim_sonar_thread[sonar_num-1].start()
        
    def read(self,offs,nbytes):
        v = None
        #v = self.__bus.read_i2c_block_data(self.__addr,offs,nbytes)
        simCode = "%2.2X%2.2X%2.2XR"%(self.__addr,offs,nbytes)
        #print ("code",simCode)
        if simCode in self.__i2cCodes.keys():
            #print ("code",simCode,"val",self.__i2cCodes[simCode])
            fn_prm = self.__i2cCodes[simCode]
            fn = fn_prm[0]
            prm = fn_prm[1]
            if len(prm) > 0:
                v = fn(prm)
            else:
                v = fn()
            #v = self.i2csim_sonar_get_version()
        else:
            print ("I2C command (code:",simCode,") is not implemented")
        return v

    def read_byte(self,offs):
        #v = self.__bus.read_byte_data(self.__addr,offs)
        v = None
        simCode = "%2.2X%2.2X01R"%(self.__addr,offs)
        #print ("code",simCode)
        if simCode in self.__i2cCodes.keys():
            #print ("code",simCode,"val",self.__i2cCodes[simCode])
            fn_prm = self.__i2cCodes[simCode]
            fn = fn_prm[0]
            prm = fn_prm[1]
            if len(prm) > 0:
                v = fn(prm)
            else:
                v = fn()
        else:
            print ("I2C command (code:",simCode,") is not implemented")
        return v
   
    def write(self,cmd,vData):
        #self.__bus.write_i2c_block_data(self.__addr,cmd,vData)
        nbytes = len(vData)
        simCode = "%2.2X%2.2X%2.2XW"%(self.__addr,cmd,nbytes)
        #print ("code",simCode)
        if simCode in self.__i2cCodes.keys():
            #print ("code",simCode,"val",self.__i2cCodes[simCode])
            fn_prm = self.__i2cCodes[simCode]
            fn = fn_prm[0]
            prm = fn_prm[1]
            for iprm in range(len(vData)):
                prm = prm +(vData[iprm],)
            if len(prm) > 0:
                #print ("prm",prm)
                fn(prm)
            else:
                fn()
        else:
            print ("I2C command (code:",simCode,") is not implemented")

    # function for I2C simulation only
    def i2cSimCodes(self):
        tCode = {}
        tCode["21C001R"] = (self.i2csim_get_vsimvar_byte,("vSonarVersion"))
        
        tCode["21B001R"] = (self.i2csim_sonar_get_mode,(1,))
        tCode["21B101R"] = (self.i2csim_sonar_get_mode,(2,))
        tCode["21B201R"] = (self.i2csim_sonar_get_mode,(3,))
        tCode["21B301R"] = (self.i2csim_sonar_get_mode,(4,))
        tCode["21B001W"] = (self.i2csim_sonar_set_mode,(1,))
        tCode["21B101W"] = (self.i2csim_sonar_set_mode,(2,))
        tCode["21B201W"] = (self.i2csim_sonar_set_mode,(3,))
        tCode["21B301W"] = (self.i2csim_sonar_set_mode,(4,))

        tCode["21B401R"] = (self.i2csim_sonar_get_state,(1,))
        tCode["21B501R"] = (self.i2csim_sonar_get_state,(2,))
        tCode["21B601R"] = (self.i2csim_sonar_get_state,(3,))
        tCode["21B701R"] = (self.i2csim_sonar_get_state,(4,))

        tCode["210002R"] = (self.i2csim_sonar_get_dist,(1,))
        tCode["210001R"] = (self.i2csim_sonar_get_dist_l,(1,))
        tCode["210101R"] = (self.i2csim_sonar_get_dist_h,(1,))
        tCode["210202R"] = (self.i2csim_sonar_get_dist,(2,))
        tCode["210201R"] = (self.i2csim_sonar_get_dist_l,(2,))
        tCode["210301R"] = (self.i2csim_sonar_get_dist_h,(2,))
        tCode["210402R"] = (self.i2csim_sonar_get_dist,(3,))
        tCode["210401R"] = (self.i2csim_sonar_get_dist_l,(3,))
        tCode["210501R"] = (self.i2csim_sonar_get_dist_h,(3,))
        tCode["210602R"] = (self.i2csim_sonar_get_dist,(4,))
        tCode["210601R"] = (self.i2csim_sonar_get_dist_l,(4,))
        tCode["210701R"] = (self.i2csim_sonar_get_dist_h,(4,))

        tCode["21A002W"] = (self.i2csim_sonar_set_dist_max,(1,))
        #tCode["21A001W"] = (self.i2csim_sonar_set_dist_max_l,(1,))
        #tCode["21A101W"] = (self.i2csim_sonar_set_dist_max_h,(1,))
        tCode["21A202W"] = (self.i2csim_sonar_set_dist_max,(2,))
        tCode["21A402W"] = (self.i2csim_sonar_set_dist_max,(3,))
        tCode["21A602W"] = (self.i2csim_sonar_set_dist_max,(4,))
        tCode["21A802W"] = (self.i2csim_sonar_set_dist_max,(5,))

        tCode["700001W"] = (self.i2csim_sonar_start_dist_diag,(5,))
        tCode["700201R"] = (self.i2csim_sonar_get_dist_diag_h,(5,))
        tCode["700301R"] = (self.i2csim_sonar_get_dist_diag_l,(5,))
        tCode["700202R"] = (self.i2csim_sonar_get_dist_diag,(5,))
        tCode["720001W"] = (self.i2csim_sonar_start_dist_diag,(6,))
        tCode["720201R"] = (self.i2csim_sonar_get_dist_diag_h,(6,))
        tCode["720301R"] = (self.i2csim_sonar_get_dist_diag_l,(6,))
        tCode["720202R"] = (self.i2csim_sonar_get_dist_diag,(6,))

        tCode["14C001R"] = (self.i2csim_get_vsimvar_byte,("vEncoderVersion"))
        tCode["140001R"] = (self.i2csim_get_vsimvar_byte_extract,("vEncoderRearRight","low"))
        tCode["140101R"] = (self.i2csim_get_vsimvar_byte_extract,("vEncoderRearRight","high"))
        tCode["140002R"] = (self.i2csim_get_vsimvar_word,("vEncoderRearRight"))
        tCode["140004R"] = (self.i2csim_get_vsimvar_word_block,(["vEncoderRearLeft","vEncoderRearRight"]))
        tCode["140201R"] = (self.i2csim_get_vsimvar_byte_extract,("vEncoderRearLeft","low"))
        tCode["140301R"] = (self.i2csim_get_vsimvar_byte_extract,("vEncoderRearLeft","high"))
        tCode["140202R"] = (self.i2csim_get_vsimvar_word,("vEncoderRearLeft"))
        tCode["140401R"] = (self.i2csim_get_vsimvar_byte,("vMotorRearDirectionRight"))
        tCode["140501R"] = (self.i2csim_get_vsimvar_byte,("vMotorRearDirectionLeft"))
        tCode["140601R"] = (self.i2csim_get_vsimvar_byte_extract,("vVoltageBin","low"))
        tCode["140701R"] = (self.i2csim_get_vsimvar_byte_extract,("vVoltageBin","high"))
        tCode["140602R"] = (self.i2csim_get_vsimvar_word,("vVoltageBin"))
        tCode["140008R"] = (self.i2csim_get_vsimvar_multi,("vEncoderAll"))
        tCode["140101W"] = (self.i2csim_encoders_reset,("right",))
        tCode["140201W"] = (self.i2csim_encoders_reset,("left",))
        tCode["140501W"] = (self.i2csim_encoders_reset,("both",))

        tCode["1E2806R"] = (self.i2csim_get_vsimvar_word_block,(["vMagX","vMagY","vMagZ"]))
        tCode["6B2806R"] = (self.i2csim_get_vsimvar_word_block,(["vAccelX","vAccelY","vAccelZ"]))
        tCode["6B2206R"] = (self.i2csim_get_vsimvar_word_block,(["vGyroX","vGyroY","vGyroZ"]))
        return tCode

    def i2csim_encoders_reset (self, vPrm):
        reset_type = vPrm[0]
        reset_val = vPrm[1]
        #print (reset_type, reset_val)
        if reset_val == 0:
            if reset_type == "left":
                self.vsv["vEncoderRearLeftReset"] = 1.0
                #print (self.vsv["vEncoderRearLeftReset"],self.vsv["vEncoderRearRightReset"])
                #print ("reset left")
            elif reset_type == "right":
                self.vsv["vEncoderRearRightReset"] = 1.0
                #print (self.vsv["vEncoderRearLeftReset"],self.vsv["vEncoderRearRightReset"])
                #print ("reset right")
            elif reset_type == "both":
                self.vsv["vEncoderRearLeftReset"] = 1.0
                self.vsv["vEncoderRearRightReset"] = 1.0
                #print (self.vsv["vEncoderRearLeftReset"],self.vsv["vEncoderRearRightReset"])
                #print ("reset both")


    def i2csim_get_vsimvar_multi(self, kym):
        w=[]
        w.append(self.i2csim_get_vsimvar_byte_extract("vEncoderRearRightLow","low"))
        w.append(self.i2csim_get_vsimvar_byte_extract("vEncoderRearRightLow","high"))
        w.append(self.i2csim_get_vsimvar_byte_extract("vEncoderRearLeftLow","low"))
        w.append(self.i2csim_get_vsimvar_byte_extract("vEncoderRearLeftLow","high"))
        w.append(self.i2csim_get_vsimvar_byte,("vMotorRearDirectionRight"))
        w.append(self.i2csim_get_vsimvar_byte,("vMotorRearDirectionLeft"))
        w.append(self.i2csim_get_vsimvar_byte_extract("vEncoderRearLeftLow","low"))
        w.append(self.i2csim_get_vsimvar_byte_extract("vEncoderRearLeftLow","high"))
        return w

    def i2csim_set_vsimvar_byte(self, vPrm):   # not tested
        ky = vPrm[0]
        val = vPrm[1]
        self.vsv[ky] = int(val)%256

    def i2csim_get_vsimvar_byte(self, ky):
        #print ("ky",ky)
        v = self.vsv[ky]
        #print ("ky",ky,"val :",v)
        w = v%256
        return w

    def i2csim_get_vsimvar_byte_extract(self, ky, lh):
        #print ("ky",ky)
        v = self.vsv[ky]
        if lh == "high":
            w = w//256
        print ("ky",ky,"L/H",lh,"val :",v)
        w = w%256
        return w

    def i2csim_get_vsimvar_word(self, ky):
        #print ("ky",ky)
        v = self.vsv[ky]
        #print ("ky",ky,"val :",v)
        w = [v%256,v//256]
        return w

    def i2csim_get_vsimvar_word_block(self, tky):
        w = []
        for ik in range(len(tky)):
            ky = tky[ik]
            #print ("ky",ky)
            v = self.vsv[ky]
            #print ("ky",ky,"val :",v)
            w.append(v%256)
            w.append(v//256)
        return w

    def i2csim_sonar_get_mode(self,sonar_num):
        ky = "vSonar%1dMode"%(sonar_num)
        return int(self.vsv[ky])

    def i2csim_sonar_set_mode(self,vPrm):
        sonar_num = vPrm[0]
        sonar_mode = vPrm[1]
        ky = "vSonar%1dMode"%(sonar_num)
        self.vsv[ky] = sonar_mode

    def i2csim_sonar_get_state(self,sonar_num):
        ky = "vSonar%1dState"%(sonar_num)
        return int(self.vsv[ky])

    def i2csim_sonar_get_dist(self,sonar_num):
        ky = "vSonar%1dDist"%(sonar_num)
        v = int(self.vsv[ky])
        w = [v%256,v//256]
        #print (ky,w)
        return w

    def i2csim_sonar_get_dist_l(self,sonar_num):
        ky = "vSonar%1dDist"%(sonar_num)
        return int(self.vsv[ky])%256

    def i2csim_sonar_get_dist_h(self,sonar_num):
        ky = "vSonar%1dDist"%(sonar_num)
        return int(self.vsv[ky])//256

    def i2csim_sonar_get_dist_diag(self,sonar_num):
        ky = "vSonar%1dDist"%(sonar_num)
        v = int(self.vsv[ky])
        w = [v//256,v%256]   # msb first
        #print ("diag",ky,w)
        return w

    def i2csim_sonar_get_dist_diag_l(self,sonar_num):
        ky = "vSonar%1dDist"%(sonar_num)
        v = int(self.vsv[ky])%256
        #print ("diagl",ky,v)
        return v

    def i2csim_sonar_get_dist_diag_h(self,sonar_num):
        ky = "vSonar%1dDist"%(sonar_num)
        v = int(self.vsv[ky])//256
        #print ("diagh",ky,v)
        return v

    def i2csim_sonar_set_dist_max (self,vPrm):
        sonar_num = vPrm[0]
        sonar_dist_max_l = vPrm[1]
        sonar_dist_max_h = vPrm[2]
        sonar_dist_max = sonar_dist_max_l + 256*sonar_dist_max_h
        if (sonar_num>0) and (sonar_num<5):
            ky = "vSonar%1dDistMax"%(sonar_num)
        else:
            ky = "vSonarDistMax"  # mode sync (2)
        self.vsv[ky] = sonar_dist_max

    def i2csim_sonar_start_dist_diag (self,vPrm):
        #print ('vPrm',vPrm) 
        sonar_num = vPrm[0]
        sonar_cmd = vPrm[1]
        ky = "vSonar%1dAcq"%(sonar_num)
        self.vsv[ky] = sonar_cmd
        #print ("vsv [",ky,"] = ", sonar_cmd)
    
    def sim_sonar(i2csm,sonar_num,vsv):
        #time.sleep(1.0)
        #print ("start sim sonar",sonar_num)
        kyDist ="vSonar%1dDist"%(sonar_num)
        kySim = "vSonar%1dSim"%(sonar_num)
        kySimEnd = "vSonar%1dSimEnd"%(sonar_num)
        if sonar_num == 1:
            kySonar = "vSonarFront"
        elif sonar_num == 2:
            kySonar = "vSonarRear"
        elif sonar_num == 3:
            kySonar = "vSonarLeft"
        elif sonar_num == 4:
            kySonar = "vSonarRight"
        kyMode ="vSonar%1dMode"%(sonar_num)
        while True:
            if vsv[kySimEnd]:
                break
            dist = vsv[kyDist]
            mode = vsv[kyMode]
            if mode != 0:
                dist = vsv[kySonar]
            if mode == 1:
                vsv[kyMode] = 0
            #print ("Sonar ",sonar_num,":",dist,"cm")
            vsv[kyDist] = dist
            time.sleep(0.001)
        vsv[kySim] = False # to tell dartv2 that end has been ackn.
        #print ("end of sim sonar",sonar_num)


    def sim_sonar_diag(i2csm,sonar_num,vsv):
        #time.sleep(1.0)
        #print ("start sim sonar diag",sonar_num)
        kyDist ="vSonar%1dDist"%(sonar_num)
        kySim = "vSonar%1dSim"%(sonar_num)
        kySimEnd = "vSonar%1dSimEnd"%(sonar_num)
        if sonar_num == 5:
            kySonar = "vSonarFrontLeft"
        elif sonar_num == 6:
            kySonar = "vSonarFrontRight"
        kyAcq ="vSonar%1dAcq"%(sonar_num)
        while True:
            if vsv[kySimEnd]:
                break
            dist = vsv[kyDist]
            acq = vsv[kyAcq]
            if acq == 0x51:
                dist = vsv[kySonar]
                vsv[kyAcq] = 0
                #print ("Sonar diag sim",sonar_num,":",dist)
            vsv[kyDist] = dist
            time.sleep(0.001)
        vsv[kySim] = False # to tell dartv2 that end has been ackn.
        #print ("end of sim sonar diag",sonar_num)
