import dartv2b
import getkey
import time

ispd = 0
tspdgo = [80,120,160]
tspdturn = [120,160,200]
idur = 0
tdur = [0.25,0.5,1.0,2.0]


def move (spdl,spdr,duration):
    mybot.set_speed (spdl,spdr)
    time.sleep(duration)
    mybot.set_speed (0,0)

def goforward():
    move (tspdgo[ispd],tspdgo[ispd],tdur[idur])

def gobackward():
    move (-tspdgo[ispd],-tspdgo[ispd],tdur[idur])

def turnleft():
    move (-tspdturn[ispd],tspdturn[ispd],tdur[idur])
 
def turnright():
    move (tspdturn[ispd],-tspdturn[ispd],tdur[idur])
   
if __name__ == "__main__":
    mybot = dartv2b.DartV2()
    getkey = getkey.GetKey()
    mybot.sonars.init_4_sonars (dmax=1.5,mode="sync")
               
    while True:
        ch = getkey()
        ich = ord(ch)
        if ich==27 or ich==3:
            break
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
        print ("speed go",tspdgo[ispd],"speed turn",tspdturn[ispd],"duration",tdur[idur])
        print ("cardinal sonars")
        print (mybot.get_cardinal_sonars())
        print ("diagonal sonars")
        print (mybot.get_diagonal_sonars())
        print ("front odometers")
        print (mybot.get_front_odos())
        print ("rear odometers")
        print (mybot.get_rear_odos())

    mybot.end() # clean end of the robot mission

