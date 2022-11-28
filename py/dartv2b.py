import drivers.dartv2b_basis
import sys
import time

# this virtual Dart is now obsolete ... to be removed

class DartV2(drivers.dartv2b_basis.DartV2Basis):
    def __init__(self):
        print ("Hi, I am Dart V2 !!!")
        # get init from parent class
        #drivers.dartv2b_basis.DartV2Basis.__init__(self)
        super().__init__()
        # define class variables

        # place your new class variables here
        self.myVarDummy = 42

    # place your new functions here
    
    def battery_voltage(self):
        return self.encoders.battery_voltage()


    def get_selected_odos(self,front_odos=False):
        if front_odos:
            odol,odor = self.get_n_mem_front_odos()
        else:
            odol,odor = self.get_n_mem_rear_odos()
        return odol,odor

    def get_n_mem_all_odos(self):
        odofl,odofr = self.get_n_mem_front_odos()
        odorl,odorr = self.get_n_mem_rear_odos()
        return odofl,odofr,odorl,odorr
    
    def get_n_mem_front_odos(self):
        """
        DART wheels are 12 cm diameter
        Virtual DART makes 200 ticks per wheel revolution
        Read from T-REX (xx ticks per revolution)
        return : odometer values (left and right) on 16 bits signed (-32768 to - 32767)
        """
        odofl,odofr = self.get_front_encoders()
        self.update_front_encoders (time.time(),odofl,odofr)
        return odofl,odofr
            
    def get_n_mem_rear_odos(self):
        odorl,odorr = self.get_rear_encoders()
        while odorl<0:
            time.sleep(0.0005)
            print ("------ error I2C odorl")
            odorl,dummy = self.get_rear_encoders()
        while odorr<0:
            time.sleep(0.0005)
            print ("------ error I2C odorr")
            dummy,odorr = self.get_rear_encoders()
        self.update_rear_encoders (time.time(),odorl,odorr)
        return odorl,odorr
    
    def delta_odometers_without_jumps (self,odo_mem,odo_last):
        deltaOdo = odo_last - odo_mem
        if deltaOdo > 32767:  # check if high positive jumps !!
            deltaOdo -= 65536 #  remove the jump (2^16)
        if deltaOdo < -32767: # same for high negative jumps
            deltaOdo += 65536 #  remove the jump (2^16)
        return deltaOdo
        
    def delta_front_odometers(self,side="both"):
        if side == "both":
            deltaOdoLeft = self.delta_odometers_without_jumps (
                self.encoders_front_left_mem,self.encoders_front_left_last)
            deltaOdoRight = self.delta_odometers_without_jumps (
                self.encoders_front_right_mem,self.encoders_front_right_last)
            return [deltaOdoLeft,deltaOdoRight]
        elif side == "left":
            deltaOdoLeft = self.delta_odometers_without_jumps (
                self.encoders_front_left_mem,self.encoders_front_left_last)
            return deltaOdoLeft
        if side == "right":
            deltaOdoRight = self.delta_odometers_without_jumps (
                self.encoders_front_right_mem,self.encoders_front_right_last)
            return deltaOdoRight
        
    def delta_rear_odometers(self,side="both"):
        if side == "both":
            deltaOdoLeft = -self.delta_odometers_without_jumps (
                self.encoders_rear_left_mem,self.encoders_rear_left_last)
            deltaOdoRight = -self.delta_odometers_without_jumps (
                self.encoders_rear_right_mem,self.encoders_rear_right_last)
            return [-deltaOdoLeft,-deltaOdoRight]
        elif side == "left":
            deltaOdoLeft = self.delta_odometers_without_jumps (
                self.encoders_rear_left_mem,self.encoders_rear_left_last)
            return -deltaOdoLeft
        if side == "right":
            deltaOdoRight = self.delta_odometers_without_jumps (
                self.encoders_rear_right_mem,self.encoders_rear_right_last)
            return -deltaOdoRight
        
    def set_sonar_0_to_99(self,d):
        if d == 0.0:
            d = 99.9
        return d
    
    def get_cardinal_sonars(self):
        df,dl,db,dr = self.sonars.read_4_sonars()
        #print ("df,dl,db,dr",df,dl,db,dr)
        df = self.set_sonar_0_to_99(df/100.0)
        dl = self.set_sonar_0_to_99(dl/100.0)
        db = self.set_sonar_0_to_99(db/100.0)
        dr = self.set_sonar_0_to_99(dr/100.0)
        self.update_cardinal_sonars(time.time(),df,dl,db,dr)
        return df,dl,db,dr

    def get_diagonal_sonars(self):
        dl,dr = self.sonars.read_diag_all()
        self.update_diagonal_sonars(time.time(),dl,dr)
        return dl,dr


if __name__ == "__main__":
    print ("start")
    if len(sys.argv) < 3:
        print ("needs to fix left and right speeds :")
        print ("python3 dart speedLeft speedRight")
        print ("exit!")
        exit()

    myDart = DartV2()

    speedL = int(sys.argv[1])
    speedR = int(sys.argv[2])

    print ("Sonar version : ",myDart.sonars.get_version())
    """
    mode = 2
    dmax = 1.5
    for isn in range(4):
        #print ("Mode",isn+1,":","%2.2X"%(myDart.sonars.get_mode(isn+1)))
        myDart.sonars.set_mode(isn+1,mode)
        myDart.sonars.set_dist_max(isn+1,dmax)
    myDart.sonars.set_dist_max(5,dmax)
    for isn in range(4):
        #print ("Mode",isn+1,":","%2.2X"%(myDart.sonars.get_mode(isn+1)))
        pass
    for isn in range(4):
        print ("State",isn+1,":","%2.2X"%(myDart.sonars.get_state(isn+1)))
    for itst in range(5):
        time.sleep(0.5)
        print ("Sonars cardinal",myDart.sonars.read_4_sonars())
        print ("Sonars front diag",myDart.sonars.read_diag_both())
        print ("Sonars front bytes",myDart.sonars.read_front_bytes())
        print ("Sonars diag word",myDart.sonars.read_diag_left_word(),myDart.sonars.read_diag_right_word())
    """
    print ("Encoder version : ",myDart.encoders.get_version())
    print ("Battery voltage :",myDart.encoders.battery_voltage())
    print ("Motor directions :",myDart.encoders.read_motors_direction())
    myDart.encoders.reset_both()
    encs_front_0 = myDart.get_front_encoders()
    encs_rear_0 = myDart.encoders.read_encoders()
    print ("Encoders (Front T-REX)) :",encs_front_0)
    print ("Encoders :",encs_rear_0)
    myDart.set_speed(speedL,speedR)
    time.sleep(1.0)
    print ("Encoders :",myDart.encoders.read_encoders())
    #myDart.encoders.reset_left()
    time.sleep(1.0)
    print ("Encoders :",myDart.encoders.read_encoders())    
    #myDart.encoders.reset_right()
    time.sleep(1.0)
    print ("Encoders :",myDart.encoders.read_encoders())    
    #myDart.encoders.reset_both()
    time.sleep(1.0)
    print ("Encoders :",myDart.encoders.read_encoders())        
    print ("Encoders (Front T-REX)) :",myDart.get_front_encoders())
    encs_front_0 = myDart.get_front_encoders()
    encs_rear_0 = myDart.encoders.read_encoders()
    print ("Encoders (Front T-REX)) :",encs_front_0)
    print ("Encoders :",encs_rear_0)
    """
    print (myDart.imu.read_mag_raw())
    """
    myDart.set_speed(0,0)
    time.sleep(0.05)
    myDart.end()
    



