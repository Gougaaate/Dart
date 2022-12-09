import drivers_v2.drivers_v2 as drv2
import time
from log import Log



if __name__ == "__main__":
    Log.dt = .1
    log = Log()

    mybot = drv2.DartV2DriverV2()
    last_delta = mybot.sonars.read_right() - mybot.sonars.read_left()

    def go_straight(last_delta):

        dt = .1
        kp, ki, kd = 0, 0, 0
        left, right = mybot.sonars.read_left(), mybot.sonars.read_right()

        if left + right < 100:
            kp, ki, kd = .25, 2.7, 0.06

        print(f"")
        delta = [last_delta, right - left]
        Log.add_to_current_data(left=left)
        Log.add_to_current_data(right=right)
        Log.add_to_current_data(P=kp*delta[1])
        Log.add_to_current_data(I=ki*(delta[0] + delta[1])*dt)
        Log.add_to_current_data(D=kd*(delta[0] - delta[1])/dt)
        print(f"{left = }, {right = }, P = {kp*delta[1]:.2f}, I = {ki*(delta[0] + delta[1])*dt:.2f}, D = {kd*(delta[0] - delta[1])/dt:.2f}")
        s = kp*delta[1] + ki*(delta[0] + delta[1])*dt + kd*(delta[0] - delta[1])/dt
        mybot.powerboard.set_speed(100 + s  , 100 - s)

        return delta[1]

        

    test = True

    while test:
        Log.current_data = []
        last_delta = go_straight(last_delta)
        Log.write_current_data()

        if mybot.sonars.read_4_sonars()[0] < 30:
            mybot.powerboard.set_speed(0, 0)
            time.sleep(1)

            if mybot.sonars.read_left() > mybot.sonars.read_right():
                mybot.turn_left()
            else:
                mybot.turn_right()


    mybot.powerboard.set_speed(0, 0)
    mybot.end()
