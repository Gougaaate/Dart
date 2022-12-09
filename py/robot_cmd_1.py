import drivers_v2.drivers_v2 as drv2
import time
from log import Log



if __name__ == "__main__":
    dt = .2
    Log.dt = dt
    log = Log()

    mybot = drv2.DartV2DriverV2()
    last_delta = mybot.sonars.read_right() - mybot.sonars.read_left()

    def go_straight(last_delta):
        kp, ki, kd = .23, 1.7, .02
        left, right = mybot.sonars.read_left(), mybot.sonars.read_right()
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
        t0_loop = time.time()
        Log.current_data = []
        last_delta = go_straight(last_delta)
        Log.write_current_data()
        # if mybot.sonars.read_4_sonars()[0] < 10:
        #     test = False

        t1_loop = time.time()
        dt_loop = t1_loop - t0_loop
        dt_sleep = dt - dt_loop
        if dt_sleep > 0:
            time.sleep(dt_sleep)

    right, left = mybot.encoders.read_encoders()

    if right < 65236:
        while mybot.encoders.read_encoders()[0] - right < 240:
            mybot.powerboard.set_speed(100, -100)

    else:
        while (mybot.encoders.read_encoders()[0] > - 65535 and mybot.encoders.read_encoders()[0] < -131071 + 240 + right) or (mybot.encoders.read_encoders()[0] > right):
            mybot.powerboard.set_speed(100, -100)
            print(mybot.encoders.read_encoders()[0])

    mybot.powerboard.set_speed(0, 0)
    mybot.end()
