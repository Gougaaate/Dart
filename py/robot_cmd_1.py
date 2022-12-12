import drivers_v2.drivers_v2 as drv2
import time
from log import Log

stop_dist = 40
mybot = drv2.DartV2DriverV2()

def go_straight(last_delta, integ):
    kp, ki, kd = .9, .6, .01
    s = 0
    left, right = mybot.sonars.read_left(), mybot.sonars.read_right()
    delta = [last_delta, right - left]

    if left + right < 60: # On est entre deux murs
        print("Entre deux murs")
        # print(f"")
        integ += ki * delta[1] * dt
        deriv = max(-10,min(kd * (delta[1] - delta[0]) / dt, 10))
        s = kp * delta[1] + integ + deriv

    elif left < 30: # On est proche du mur gauche
        print("Proche du mur gauche")
        delta = [last_delta, 21 - left]
        integ += ki * delta[1] * dt
        deriv = max(-10,min(kd * (delta[1] - delta[0]) / dt, 10))
        s = kp * delta[1] + integ + deriv

    elif right < 30: # On est proche du mur droit
        print("Proche du mur droit")
        delta = [last_delta, right - 21]
        integ += ki * delta[1] * dt
        deriv = max(-10,min(kd * (delta[1] - delta[0]) / dt, 10))
        s = kp * delta[1] + integ + deriv
    
    else:
        s = 0

    mybot.powerboard.set_speed(80 + s, 80 - s)

    Log.add_to_current_data(left=left, right=right)
    Log.add_to_current_data(front=mybot.sonars.read_4_sonars()[0])
    Log.add_to_current_data(stop_dist=stop_dist)
    Log.add_to_current_data(P=kp * delta[1], I=integ, D=kd * (delta[1] - delta[0]) / dt)

    return delta[1], integ


def turn_right(mybot):
    mybot.powerboard.set_speed(80,-80)
    time.sleep(1)
    mybot.powerboard.set_speed(0,0)

def turn_left(mybot):
    mybot.powerboard.set_speed(-80, 80)
    time.sleep(1)
    mybot.powerboard.set_speed(0,0)


def stop(mybot):
    print("stop", time.time() - t_init, "s")
    t0 = time.time()
    aimed_dist = 35
    k = 1
    offset = 20

    dist = mybot.sonars.read_4_sonars()[0]
    while abs(dist - aimed_dist) > 10 and time.time() - t0 < 4:
        t0_loop = time.time()
        dist = mybot.sonars.read_4_sonars()[0]
        s = min(50,max(k*(dist - aimed_dist) + offset,-50))
        mybot.powerboard.set_speed(s, s)

        t1_loop = time.time()
        dt_loop = t1_loop - t0_loop
        dt_sleep = dt - dt_loop
        if dt_sleep > 0:
            time.sleep(dt_sleep)
    
    print("sorti")
    mybot.powerboard.set_speed(0, 0)
    time.sleep(.5)


if __name__ == "__main__":
    dt = .1
    Log.dt = dt
    log = Log()

    mybot = drv2.DartV2DriverV2()
    last_delta = mybot.sonars.read_right() - mybot.sonars.read_left()

    test = True
    integ = 0
    t_init = time.time()
    while test:
        t0_loop = time.time()
        Log.current_data = []
        last_delta, integ = go_straight(last_delta, integ)
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
