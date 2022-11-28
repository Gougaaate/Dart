import drivers_v2.drivers_v2 as drv2
import dartv2_control 
import getkey
import time

# control DART robot from the keyboard

ispd = 0
tspdgo = [80,120,160]
tspdturn = [120,160,200]
idur = 0
tdur = [0.25,0.5,1.0,2.0]


def move (spdl,spdr,duration):
    mybot.powerboard.set_speed (spdl,spdr)
    time.sleep(duration)
    mybot.powerboard.set_speed (0,0)

def goforward():
    move (tspdgo[ispd],tspdgo[ispd],tdur[idur])

def gobackward():
    move (-tspdgo[ispd],-tspdgo[ispd],tdur[idur])

def turnleft():
    move (-tspdturn[ispd],tspdturn[ispd],tdur[idur])
 
def turnright():
    move (tspdturn[ispd],-tspdturn[ispd],tdur[idur])
   
if __name__ == "__main__":
    mybot = drv2.DartV2DriverV2()
    mybot_ctrl = dartv2_control.DartV2Control(mybot)
    get_key = getkey.GetKey()
    my_system = get_key.system
    mybot.sonars.init_4_sonars (dmax=1.2,mode="sync")
               
    while True:
        print ("Usage : (g)go, (b)back, (f)left, (h)right")
        ch = get_key()
        ich = ord(ch)
        if ch != 0:
            #print (ch,ich,ch==b'b')
            if ich==27 or ich==3:
                break
            if my_system == "Windows":
                if ch==b"g":
                    goforward()
                elif ch==b"b":
                    gobackward()
                elif ch==b"f":
                    turnleft()
                elif ch==b"h":
                    turnright()
                elif ch==b"1":
                    ispd=0
                elif ch==b"2":
                    ispd=1
                elif ch==b"3":
                    ispd=2
                elif ch==b"4":
                    idur=0
                elif ch==b"5":
                    idur=1
                elif ch==b"6":
                    idur=2
                elif ch==b"7":
                    idur=3
            elif my_system == "Linux":
                if ch=="g":
                    goforward()
                elif ch=="b":
                    gobackward()
                elif ch=="f":
                    turnleft()
                elif ch=="h":
                    turnright()
                elif ch=="1":
                    ispd=0
                elif ch=="2":
                    ispd=1
                elif ch=="3":
                    ispd=2
                elif ch=="4":
                    idur=0
                elif ch=="5":
                    idur=1
                elif ch=="6":
                    idur=2
                elif ch=="7":
                    idur=3
            else:
                pass 
            print ("speed go",tspdgo[ispd],"speed turn",tspdturn[ispd],"duration",tdur[idur])
            print ("cardinal sonars")
            print (mybot.sonars.read_4_sonars())
            print ("diagonal sonars")
            print (mybot.sonars.read_diag_both())
            print ("front odometers")
            print (mybot_ctrl.get_front_encoders())
            print ("rear odometers")
            print (mybot_ctrl.get_rear_encoders())

    mybot.end() # clean end of the robot mission

