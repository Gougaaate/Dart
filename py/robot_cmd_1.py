import dartv2b
import time
if __name__ == "__main__":

    mybot = dartv2b.DartV2()

    test = True
    while test:
        print(mybot.get_cardinal_sonars()[0])
        mybot.set_speed(100,100)
        if mybot.get_cardinal_sonars()[0] < 10:
            test =False
    # right, left = mybot.get_front_encoders()
    # if right < 65236:
    #     while mybot.get_front_encoders()[0] - right < 240:
    #         mybot.set_speed(100,-100)
    #
    # else:
    #     while (mybot.get_front_encoders()[0] > - 65535 and mybot.get_front_encoders()[0] < -131071 + 240 + right) or (mybot.get_front_encoders()[0]  > right) :
    #         mybot.set_speed(100, -100)
    #         print (mybot.get_front_encoders()[0])

    mybot.set_speed(0,0)
    mybot.end()


