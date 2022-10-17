import time
import sys
import os

# rear wheels encoders and direction, battery level
# encoders reset

class EncodersIO ():
    def __init__(self, bus_nb = 2, addr = 0x14, sim=False, vsv=None):
        self.__sim = sim
        if self.__sim:
            self.vsv = vsv

        self.__bus_nb = 2
        self.__addr = 0x14 

        # conditional i2c setup
        # if real robot , then we use actual i2c
        # if not , we are on simulated i2c
        if self.__sim:
            import i2csim as i2c
            self.__dev_i2c=i2c.i2c(self.__addr,self.__bus_nb,vsv=self.vsv)
        else:
            import i2creal as i2c
            self.__dev_i2c=i2c.i2c(self.__addr,self.__bus_nb)

        # place your new class variables here

        # initialize  measurements in memory
        self.enc_left_last = 0
        self.enc_right_last = 0
        self.enc_time_last = time.time() # time my be used for  speed estimation
        self.enc_time_mem = self.enc_time_last
        self.enc_left_mem = self.enc_left_last
        self.enc_right_mem = self.enc_right_last
        self.read_encoders() # initialize encoders memory
        
        
    # place your encoder functions here
      
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
        # put last measurement in memory
        self.enc_left_mem  = self.enc_left_last
        self.enc_right_mem  = self.enc_right_last
        self.enc_time_mem = self.enc_time_last

        # do new measurement
        self.enc_time_last = time.time()
        offs = 0
        self.enc_left = self.__read(offs)
        offs = 2
        self.enc_right = self.__read(offs)
        self.enc_left_last = self.enc_left
        self.enc_right_last = self.enc_right
        return [self.enc_left, self.enc_right]

    def read_encoders_both (self):
        # do new measurement
        self.enc_time_last = time.time()
        offs = 0
        i2c_ok = True
        try:
            v = self.__dev_i2c.read(offs,4)
        except:
            i2c_ok = False
        if i2c_ok:
            # put last measurement in memory
            self.enc_left_mem  = self.enc_left_last
            self.enc_right_mem  = self.enc_right_last
            self.enc_time_mem = self.enc_time_last
            # compute and store new measurement
            self.enc_left =  v[0] + (v[1] << 8)
            self.enc_right =  v[2] + (v[3] << 8)
            self.enc_left_last = self.enc_left
            self.enc_right_last = self.enc_right
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
    # ... so here we can only test statically with the values in
    #     vDartv2/vSimVar.py
    
    # test if on real robot , if gpio exists (not very robust)
    # add test on processor type, real robot has armv7l
    tstsim = False
    if (os.access("/sys/class/gpio/gpio266", os.F_OK)) \
       and (platform.processor() == 'armv7l'):
        encoders = EncodersIO()
        print ("Work with real DART")
    # if not the virtual robot is running in V-REP
    else :
        tstsim = True
        sys.path.append('../../vDartV2')
        import vSimVar as vsv
        encoders = EncodersIO(sim=True,vsv=vsv.tSimVar)
        print ("Work with virtual DART on V-REP")

    print ("Encoder Microcode Version : ",encoders.get_version())
    print ("Battery Voltage (V) :",encoders.battery_voltage())
    print ("Encoders :",encoders.read_encoders())
    print ("Motors direction :",encoders.read_motors_direction())
    print ("Reset Encoders")
    encoders.reset_both()
