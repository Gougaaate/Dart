import drivers_v2.drivers_v2 as drv2
import time
from log import Log

stop_dist = 60

def go_straight(last_delta):
    kp, ki, kd = 0, 0, 0
    left, right = mybot.sonars.read_left(), mybot.sonars.read_right()


    if left + right < 60:
        kp, ki, kd = 0.7, 0.8, 0.3
        # print(f"")
    delta = [last_delta, right - left]
    Log.add_to_current_data(left=left)
    Log.add_to_current_data(right=right)
    Log.add_to_current_data(front=mybot.sonars.read_4_sonars()[0])
    Log.add_to_current_data(stop_dist=stop_dist)
    Log.add_to_current_data(P=kp * delta[1])
    Log.add_to_current_data(I=ki * (delta[0] + delta[1]) * dt)
    Log.add_to_current_data(D=kd * (delta[0] - delta[1]) / dt)
    # print(f"{left = }, {right = }, P = {kp * delta[1]:.2f}, I = {ki * (delta[0] + delta[1]) * dt:.2f}, D = {kd * (delta[0] - delta[1]) / dt:.2f}")
    s = kp * delta[1] + ki * (delta[0] + delta[1]) * dt - kd * (delta[0] - delta[1]) / dt
    mybot.powerboard.set_speed(100 + s, 100 - s)

    return delta[1]


def turn_right(mybot):
    mybot.powerboard.set_speed(-50,-50)
    time.sleep(0.5)
    mybot.powerboard.set_speed(0,0)
    time.sleep(0.5)

    mybot.powerboard.set_speed(80,-80)
    time.sleep(0.95)
    mybot.powerboard.set_speed(0,0)

def turn_left(mybot):
    mybot.powerboard.set_speed(-50, -50)
    time.sleep(0.5)
    mybot.powerboard.set_speed(0, 0)
    time.sleep(0.5)

    mybot.powerboard.set_speed(-80, 80)
    time.sleep(0.95)
    mybot.powerboard.set_speed(0,0)


def stop(mybot):
    print("stop", time.time() - t_init, "s")
    aimed_dist = 25
    k = 1
    offset = 20

    dist = mybot.sonars.read_4_sonars()[0]
    s = 16
    while abs(dist - aimed_dist) > 5:
        t0_loop = time.time()
        dist = mybot.sonars.read_4_sonars()[0]
        s = k*(dist - aimed_dist) + offset
        mybot.powerboard.set_speed(s, s)

        dt_loop = t1_loop - t0_loop
        dt_sleep = dt - dt_loop
        if dt_sleep > 0:
            time.sleep(dt_sleep)
    
    mybot.powerboard.set_speed(0, 0)
    time.sleep(1.5)


if __name__ == "__main__":
    dt = .1
    Log.dt = dt
    log = Log()

    mybot = drv2.DartV2DriverV2()
    last_delta = mybot.sonars.read_right() - mybot.sonars.read_left()

    test = True
    t_init = time.time()
    while test:
        t0_loop = time.time()
        Log.current_data = []
        last_delta = go_straight(last_delta)
        Log.write_current_data()

        if mybot.sonars.read_4_sonars()[0] < stop_dist:
            stop(mybot)

            if mybot.sonars.read_left() > mybot.sonars.read_right():
                print("turn left")
                turn_left(mybot)
                time.sleep(0.3)

            else:
                print("turn right")
                turn_right(mybot)
                time.sleep(0.3)

        t1_loop = time.time()
        dt_loop = t1_loop - t0_loop
        dt_sleep = dt - dt_loop
        if dt_sleep > 0:
            time.sleep(dt_sleep)

    mybot.powerboard.set_speed(0, 0)
    mybot.end()
