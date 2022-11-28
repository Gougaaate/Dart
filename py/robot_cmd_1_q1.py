import drivers_v2.drivers_v2 as drv2
import time


if __name__ == "__main__":
    mybot = drv2.DartV2DriverV2()
    mybot.powerboard.set_speed(80,80)
    time.sleep(5)
    # mybot.sonars.init_4_sonars()
    # test = True
    # while test:
    #     mybot.powerboard.set_speed(80,80)
    #     print(f"front = {mybot.sonars.read_4_sonars()[0]} left = {mybot.sonars.read_4_sonars()[1]} right = {mybot.sonars.read_4_sonars()[3]}")
    #     if mybot.sonars.read_4_sonars()[0] < 60:
    #         time.sleep(1)
    #         if mybot.sonars.read_4_sonars()[1] <= mybot.sonars.read_4_sonars()[3]:
    #             print("turn right")
    #             mybot.turn_right()
    #         else:
    #             print("turn left")
    #             mybot.turn_left()


    mybot.powerboard.set_speed(0,0)
    mybot.end()


