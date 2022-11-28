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
import vSimVar as vSimVar
import time

class i2c():
    def __init__(self,addr,bus_nb=2):
        self.__addr_rear_enc = 0x14
        self.__addr_four_sonars = 0x21
        self.__addr_diag_left = 0x70
        self.__addr_diag_right = 0x72
        self.__addr_imu9_mg = 0x1e
        self.__addr_imu9_ag = 0x6b

        self.__devices_wr=[self.__addr_four_sonars,
                           self.__addr_diag_left,
                           self.__addr_diag_right,
                           self.__addr_imu9_mg,
                           self.__addr_imu9_ag]

        self.__devices_rd=[self.__addr_rear_enc,
                           self.__addr_four_sonars,
                           self.__addr_diag_left,
                           self.__addr_diag_right,
                           self.__addr_imu9_mg,
                           self.__addr_imu9_ag]

        self.__bus_nb = bus_nb
        self.__addr = addr
        #self.__bus = smbus.SMBus(self.__bus_nb)
        print ("i2c device addr = 0x%x on bus %d"%(addr,bus_nb))
    
    def read (self,offs,nbytes):
        if self.__addr in self.__devices_rd:
            addrOk = False
            cmdOk = False
            if self.__addr ==  self.__addr_four_sonars:
                addrOk = True
                cmdOk = False
                if offs == 0x01 and nbytes == 2: # sonar front (1)
                    cmdOk = True
                    v = self.vSonarToTwoBytes (vSimVar.sonar_front)
                elif offs == 0x02 and nbytes == 2: # sonar rear (2)
                    cmdOk = True
                    v = self.vSonarToTwoBytes (vSimVar.sonar_rear)
                elif offs == 0x03 and nbytes == 2: # sonar left (3)
                    cmdOk = True
                    v = self.vSonarToTwoBytes (vSimVar.sonar_left)
                elif offs == 0x04 and nbytes == 2: # sonar right (4)
                    cmdOk = True
                    v = self.vSonarToTwoBytes (vSimVar.sonar_right)            
            elif self.__addr ==  self.__addr_rear_enc:
                addrOk = True
                cmdOk = False
                if offs == 0x01 and nbytes == 2: # left encoder (1)
                    cmdOk = True
                    v = self.vEncoderToTwoBytes (vSimVar.odo_rear_left)
                elif offs == 0x02 and nbytes == 2: # right encoder (2)
                    cmdOk = True
                    v = self.vEncoderToTwoBytes (vSimVar.odo_rear_right)
                elif offs == 0x03 and nbytes == 2: # left motor direction (3)
                    cmdOk = True
                    v = self.vEncoderToTwoBytes (vSimVar.motor_direction_left)
                elif offs == 0x04 and nbytes == 2: # right moto direction (4)
                    cmdOk = True
                    v = self.vEncoderToTwoBytes (vSimVar.motor_direction_right)
                elif offs == 0x05 and nbytes == 2: # battery level (5)
                    cmdOk = True
                    v = self.vBatteryToTwoBytes (vSimVar.voltage)
            elif self.__addr ==  self.__addr_imu9_mg:
                addrOk = True
                cmdOk = False
                if offs == 0x28 and nbytes == 6: # 6 bytes mag (x,y,z)
                    cmdOk = True
                    v = [0,0,0,0,0,0,0]
                    # check vSimVar.imu9_config_mg == 0x0F
                    # to be implemented from vHeading
                    # v = self.vHeadingToSixBytes (vSimVar.heading)
            elif self.__addr ==  self.__addr_imu9_ag:
                addrOk = True
                cmdOk = False
                if offs == 0x22 and nbytes == 6: # 6 bytes gyro (x,y,z)
                    cmdOk = True
                    v = [0,0,0,0,0,0,0]
                    # check vSimVar.imu9_config_ag == 0x0F
                    # to be implemented from gyros
                    # v = self.vHeadingToSixBytes (vSimVar.gyros)
                if offs == 0x28 and nbytes == 6: # 6 bytes accel (x,y,z)
                    cmdOk = True
                    v = [0,0,0,0,0,0,0]
                    # check vSimVar.imu9_config_ag == 0x0F
                    # to be implemented from accelleros
                    # v = self.vHeadingToSixBytes (vSimVar.accels)
            if addrOk is False:
                print ("I2C read address 0x%x not implemented"%(self.__addr))
                v = None
            elif cmdOk is False:
                print ("I2C read command 0x%x not valid (@ address 0x%x)"%(offs,self.__addr))
                v = None
        else:
            print ("I2C read address 0x%x not valid"%(self.__addr))
            v = None
        return v

    
    def read_byte (self,offs):
        #print ("I2C read byte 0x%x 0x%x"%(self.__addr,offs))
        dtMin = 0.03
        if self.__addr in self.__devices_rd:
            addrOk = False
            cmdOk = False
            if self.__addr ==  self.__addr_diag_left:
                addrOk = True
                cmdOk = False
                if offs == 0x02: # sonar left front msbyte (2)
                    cmdOk = True
                    dt = time.time()-vSimVar.trig_time_left
                    #print ("dt = ",dt)
                    if vSimVar.trig_left and dt>dtMin:
                        v=self.vSonarToByte(vSimVar.sonar_front_left,1)
                        vSimVar.reset_trig_left |= 0x01
                        if vSimVar.reset_trig_left == 0x03:
                            vSimVar.trig_left = False
                            vSimVar.reset_trig_left = 0
                    else:
                        v=0
                elif offs == 0x03: # sonar left front lsbyte (3)
                    cmdOk = True
                    dt = time.time()-vSimVar.trig_time_left
                    #print ("dt = ",dt)
                    if vSimVar.trig_left and dt>dtMin:
                        v=self.vSonarToByte(vSimVar.sonar_front_left,0)
                        vSimVar.reset_trig_left |= 0x02
                        if vSimVar.reset_trig_left == 0x03:
                            vSimVar.trig_left = False
                            vSimVar.reset_trig_left = 0
                    else:
                        v=0
            if self.__addr ==  self.__addr_diag_right:
                addrOk = True
                cmdOk = False
                if offs == 0x02: # sonar right front msbyte (2)
                    cmdOk = True
                    dt = time.time()-vSimVar.trig_time_right
                    #print ("dt = ",dt)
                    if vSimVar.trig_right and dt>dtMin:
                        v=self.vSonarToByte(vSimVar.sonar_front_right,1)
                        vSimVar.reset_trig_right |= 0x01
                        #print ("reset_trig_right bit 1",
                        #       vSimVar.reset_trig_right)
                        if vSimVar.reset_trig_right == 0x03:
                            vSimVar.trig_right = False
                            vSimVar.reset_trig_right = 0
                    else:
                        v=0
                elif offs == 0x03: # sonar right front lsbyte (3)
                    cmdOk = True
                    dt = time.time()-vSimVar.trig_time_right
                    #print ("dt = ",dt)
                    if vSimVar.trig_right and dt>dtMin:
                        v=self.vSonarToByte(vSimVar.sonar_front_right,0)
                        vSimVar.reset_trig_right |= 0x02
                        #print ("reset_trig_right bit 2",
                        #       vSimVar.reset_trig_right)
                        if vSimVar.reset_trig_right == 0x03:
                            vSimVar.trig_right = False
                            vSimVar.reset_trig_right = 0
                    else:
                        v=0
            if addrOk is False:
                print ("I2C read byte address 0x%x not implemented"%(self.__addr))
                v = None
            elif cmdOk is False:
                print ("I2C read byte command 0x%x not valid (@ address 0x%x)"%(offs,self.__addr))
                v = None
        else:
            print ("I2C read byte address 0x%x not valid"%(self.__addr))
            v = None
        return v
  
    def write(self,cmd,vData):
        #print ("I2C write 0x%x 0x%x"%(self.__addr,cmd),vData)
        if self.__addr in self.__devices_wr:
            addrOk = False
            cmdOk = False
            if self.__addr ==  self.__addr_four_sonars:
                addrOk = True
                cmdOk = False
                if cmd == 0x00 and len(vData) == 2: # sonar cmd 
                    if vData[0] == 0 and vData[1] == 0x10: # 100 ms
                        cmdOk = True
                        vSimVar.sonar_delay = 0.1
                    elif vData[0] == 0 and vData[1] == 0x20: # 200 ms
                        cmdOk = True
                        vSimVar.sonar_delay = 0.2
                    elif vData[0] == 0 and vData[1] == 0x30: # 300 ms
                        cmdOk = True
                        vSimVar.sonar_delay = 0.3
                    elif vData[0] == 0 and vData[1] == 0x40: # 400 ms
                        cmdOk = True
                        vSimVar.sonar_delay = 0.4
                    elif vData[0] == 0 and vData[1] == 0x50: # 500 ms
                        cmdOk = True
                        vSimVar.sonar_delay = 0.5
                    elif vData[0] == 0 and vData[1] == 0x65: # sonar front
                        cmdOk = True
                        vSimVar.sonar_config = "Front"
                    elif vData[0] == 0 and vData[1] == 0x66: # sonar rear
                        cmdOk = True
                        vSimVar.sonar_config = "Rear"
                    elif vData[0] == 0 and vData[1] == 0x67: # sonar left
                        cmdOk = True
                        vSimVar.sonar_config = "Left"
                    elif vData[0] == 0 and vData[1] == 0x68: # sonar right
                        cmdOk = True
                        vSimVar.sonar_config = "Right"
                    elif vData[0] == 0 and vData[1] == 0xFF: # all 4 sonars
                        cmdOk = True
                        vSimVar.sonar_config = "All"
            elif self.__addr ==  self.__addr_diag_left:
                addrOk = True
                cmdOk = False
                if cmd == 0x00 and len(vData) == 1: # sonar cmd 
                    if vData[0] == 0x51: # trigger
                        cmdOk = True
                        vSimVar.trig_left = True
                        vSimVar.trig_time_left = time.time()
                        vSimVar.reset_trig_left = 0
            elif self.__addr ==  self.__addr_diag_right:
                addrOk = True
                cmdOk = False
                if cmd == 0x00 and len(vData) == 1: # sonar cmd 
                    if vData[0] == 0x51: # trigger
                        cmdOk = True
                        vSimVar.trig_right = True
                        vSimVar.trig_time_right = time.time()
                        vSimVar.reset_trig_right = 0
            elif self.__addr ==  self.__addr_imu9_mg:
                addrOk = True
                cmdOk = False
                if cmd == 0x20 and len(vData) == 1:  # CTRL_REG1
                    cmdOk = True
                    if vData[0] == 0x70:
                        vSimVar.imu9_config_mg |= 1
                    else:
                        vSimVar.imu9_config_mg &= 0xFE
                elif cmd == 0x21 and len(vData) == 1:  # CTRL_REG2
                    cmdOk = True
                    if vData[0] == 0x00:
                        vSimVar.imu9_config_mg |= 2
                    else:
                        vSimVar.imu9_config_mg &= 0xFD
                elif cmd == 0x22 and len(vData) == 1:  # CTRL_REG3
                    cmdOk = True
                    if vData[0] == 0x00:
                        vSimVar.imu9_config_mg |= 4
                    else:
                        vSimVar.imu9_config_mg &= 0xFB
                elif cmd == 0x23 and len(vData) == 1:  # CTRL_REG4
                    cmdOk = True
                    if vData[0] == 0x0C:
                        vSimVar.imu9_config_mg |= 8
                    else:
                        vSimVar.imu9_config_mg &= 0xF7                        
            elif self.__addr ==  self.__addr_imu9_ag:
                addrOk = True
                cmdOk = False
                if cmd == 0x11 and len(vData) == 1:  # gyro 1
                    cmdOk = True
                    if vData[0] == 0x8C: 
                        vSimVar.imu9_config_ag |= 1
                    else:
                        vSimVar.imu9_config_ag &= 0xFE
                elif cmd == 0x16 and len(vData) == 1:  # gyro 2
                    cmdOk = True
                    if vData[0] == 0x00:
                        vSimVar.imu9_config_ag |= 2
                    else:
                        vSimVar.imu9_config_ag &= 0xFD
                elif cmd == 0x10 and len(vData) == 1:  # acc 1
                    cmdOk = True
                    if vData[0] == 0x8C:
                        vSimVar.imu9_config_ag |= 4
                    else:
                        vSimVar.imu9_config_ag &= 0xFB
                elif cmd == 0x12 and len(vData) == 1:  # common
                    cmdOk = True
                    if vData[0] == 0x04:
                        vSimVar.imu9_config_ag |= 8
                    else:
                        vSimVar.imu9_config_ag &= 0xF7                        
            if addrOk is False:
                print ("I2C write address 0x%x not implemented"%(self.__addr))
                v = None
            elif cmdOk is False:
                print ("I2C write data ",vData)
                print ("I2C write command 0x%x not valid (@ address 0x%x)"%(cmd,self.__addr))
                v = None
        else:
            print ("I2C write address 0x%x not valid"%(self.__addr))
            v = None



    def vSonarToByte(self,v,lh):
        iv = int(v*100)
        if lh != 0:
            ivb = ((iv & 0xFF00) >> 8) & 0x00FF
        else:
            ivb = (iv & 0x00FF)
        return ivb
            
    def vEncoderToTwoBytes (self,v):
        iv = int(v)
        ivh = ((iv & 0xFF00) >> 8) & 0x00FF
        ivl = (iv & 0x00FF)
        return [ivl,ivh]

    def vSonarToTwoBytes (self,v):
        iv = int(v*100)
        ivh = ((iv & 0xFF00) >> 8) & 0x00FF
        ivl = (iv & 0x00FF)
        return [ivl,ivh]

    def vBatteryToTwoBytes (self,v):
        iv = int(v*0.23*1024./5.0)
        ivh = ((iv & 0xFF00) >> 8) & 0x00FF
        ivl = (iv & 0x00FF)
        return [ivl,ivh]
