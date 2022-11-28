#!/usr/bin/python
# -*- coding: utf-8 -*-

# Initial release 12/05/2016, rewrite of DART/Trex interface
# © I. Probst, ENSTA-Bretagne, WTFPL

# updated benblop 2021/12/15 : add 4 exec_robot modes
#   "Sim V-REP" "Sim GAZEBO" "Real" "Real ROS":

import struct
from collections import OrderedDict
import time
import sys
import os

# from stackoverflow, it's safer to proxy dict rather than subclassing it
# this way we are sure to give access to methods we know we want to provide
# rather than falling back to parent class' methods



# provide a student-proof dictionary with immutable keys 
class FixedDict():
        def __init__(self, dictionary):
            self._dictionary = dictionary
        def __setitem__(self, key, item):
                if key not in self._dictionary:
                    raise KeyError("The key {} is not defined.".format(key))
                self._dictionary[key] = item
        def __getitem__(self, key):
            return self._dictionary[key]
        def values(self):
            return self._dictionary.values()

class TrexIO():
    """
    Handles communications with Trex power board.
    This class exports two dict called: ``command`` and ``status`` (see below)
    mapped to the corresponding bytes in the binary packet used for I/O with
    the Trex board.
    
    Parameters
    ----------
    bus_nb : integer, optionnal, defaults to 2
        I2C bus to use
    addr : integer, optionnal, defaults to 0x7
        I2C address of the Trex Board
        
    Notes
    -----

    ``status`` is mapped as follows:
        'start_byte'                    Start byte – will be 0xF0 (240 decimal)
        'error_flag'                    Error flag – 0 = ok
        'battery'                       Battery voltage 
        'lm_current'                    Left motor current
        'lm_enc'                        Left encoder count
        'rm_current'                    Right motor current
        'rm_enc'                        Right motor encoder
        'acc_x'                         Accelerometer X-axis
        'acc_y'                         Accelerometer Y-axis
        'acc_z'                         Accelerometer Z-axis
        'impact_x'                      Impact X-axis
        'impact_y'                      Impact Y-axis
        'impact_z'                      Impact Z-axis

    ``status`` is mapped as follows:
        'start_byte'                    Start byte - must be 0x0F (15 decimal)
        'pwm_freq'                      PWMfreq
        'lm_speed_high_byte'            Left speed high byte
        'lm_speed_low_byte'             Left Speed low byte
        'lm_brake'                      Left brake
        'rm_speed_high_byte'            Right Speed high byte
        'rm_speed_low_byte'             Right Speed low byte
        'rm_brake'                      Right brake
        'servo_1_high_byte'             Servo 1 high byte
        'servo_1_low_byte'              Servo 1 low byte
        'servo_2_high_byte'             Servo 2 high byte
        'servo_2_low_byte'              Servo 2 low byte
        'servo_3_high_byte'             Servo 3 high byte
        'servo_3_low_byte'              Servo 3 low byte
        'servo_4_high_byte'             Servo 4 high byte
        'servo_4_low_byte'              Servo 4 low byte
        'servo_5_high_byte'             Servo 5 high byte
        'servo_5_low_byte'              Servo 5 low byte
        'servo_6_high_byte'             Servo 6 high byte
        'servo_6_low_byte'              Servo 6 low byte
        'devibrate'                     Devibrate
        'impact_sensitivity_high_byte'  Impact sensitivity high byte
        'impact_sensitivity_low_byte'   Impact sensitivity low byte
        'battery_high_byte'             Battery voltage high byte (motors off)
        'battery_low_byte'              Battery voltage low byte (motors off)
        'i2c_address'                   I2C slave address
        'i2c_clock'                     I2C clock frequency
        
    """
     
    __command_dict_strings = ('left_motor_speed', 
                              'right_motor_speed', 'use_pid', 'crc')
                
 
    __nb_command_bytes = 6
        
    __status_dict_strings = (           
                'left_encoder', 'right_encoder',
                '__dont_use_this_padding_byte', 'crc'
                )
                
    __nb_status_bytes = 6
    
    __I2C_SLAVE=0x0703
    __I2C_TENBIT=0x0704


    def __init__(self, bus_nb = 2, addr = 0x07, exec_robot="Sim V-REP"):
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
        self.__sim = self.__dartSimVrep or self.__dartSimGazebo
        
        if self.__sim:
            pass
        else:
            import crcmod
            import fcntl
            self.__i2c_fd ={
                    "in": open("/dev/i2c-"+str(bus_nb), 'rb', buffering=0),
                    "out": open("/dev/i2c-"+str(bus_nb), 'wb', buffering=0)
            }
            for fd in self.__i2c_fd.values():
                fcntl.ioctl(fd, TrexIO.__I2C_TENBIT, 0)
                if fcntl.ioctl(fd, TrexIO.__I2C_SLAVE, 7) < 0:
                    raise IOError(
                        "Can not find a T-rex at address %d on bus %d"%(addr, bus_nb)
                    )
                       
        self.__reset_bytes = (0,0,0,0)

        # this list will be used as a "pointer of pointers" to speed up
        # the dict updates

                    

                    
        self.__status = dict(
                    zip(
                        TrexIO.__status_dict_strings, 
                        [0]*len(TrexIO.__status_dict_strings)
                        )
                    )
        
        if not self.__sim:
            self.compute_crc = crcmod.mkCrcFun(0x131, initCrc=0) #Dallas polynom
        self.reset()
        
    @property
    def status(self):
        if not self.__sim:
            raw_status = self.i2c_read()
            # I found no way to get around this as dicts are built with copies
            # of the values...
            
            data = struct.unpack("<hhBB", raw_status)
            crc = self.compute_crc(raw_status[:-1], 0)
            if data[-1] != crc:
                self.reset()
                raise ValueError(
                    "Expected crc %d got %d.\n Trying to stop motors"%
                    (data[-1], crc)
                    )

            self.__status.update(
                        zip(
                            TrexIO.__status_dict_strings, 
                            data
                            )
                        )

        return self.__status
    
    def reset(self):
        self.command = FixedDict(OrderedDict(
                zip(
                TrexIO.__command_dict_strings,
                self.__reset_bytes)
                ))
        if self.__sim:
           pass
        else:
            self.i2c_write()

    def i2c_read(self):
        '''
        Read status from Trex board.
        Warning, this function does not perform any sanity check on the values
        got from the board.

        Parameters
        ----------
        None

        Return values
        -------------
        data_packet: bytes
            Raw data packet read from i2c        
        '''
        data_packet = None
        if self.__sim:
           pass
        else:        
           data_packet = self.__i2c_fd["in"].read(TrexIO.__nb_status_bytes)
        return data_packet
        
    def i2c_write(self):
        '''
        Write bytes from ``command`` to the Trex board.
        Warning, this function does not perform any sanity check on the values
        sent to the board. .
        Parameters
        ----------
        None

        Return values
        -------------
        None
        
        '''

        # extract all data from command dict, ignore crc which should not
        # be modified by the user anyway
        data_packet = struct.pack(
                                "<hhB",
                                *list(self.command.values())[:-1]
                                )

        if self.__sim:
            pass
        else:  
            crc = self.compute_crc(data_packet, 0)              
            self.__i2c_fd["out"].write(data_packet+struct.pack("B", crc))
            self.__i2c_fd["out"].flush()
            # throttle down students...
            # time.sleep(1e-3)

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
        trex = TrexIO("Sim V-REP")
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


    
    print("Testing motors")
    print(trex.status)
    time.sleep(1e-3)
    trex.command["left_motor_speed"] = val_left
    trex.command["right_motor_speed"]= val_right
    trex.i2c_write()
    time.sleep(duration)
    trex.command["left_motor_speed"]= 0
    trex.command["right_motor_speed"]= 0
    trex.i2c_write()
    print(trex.status)

    if tstsim:
        tSimVar["vSimAlive"] = False
