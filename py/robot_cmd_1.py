import drivers_v2.drivers_v2 as drv2
import time
from log import Log

stop_dist = 50
mybot = drv2.DartV2DriverV2()


def borne(x, b):
    return max(-abs(b), min(abs(b), x))


def go_straight(last_delta, integ):
    kp, ki, kd = .10, .65, .01
    s, prop, deriv = 0, 0, 0
    left, right = mybot.sonars.read_left(), mybot.sonars.read_right()
    delta = [last_delta, right - left]

    if left + right < 60:  # On est entre deux murs
        print("Entre deux murs")
        # print(f"")
        prop = kp * delta[1]
        integ += ki * delta[1] * dt
        deriv = kd * (delta[1] - delta[0]) / dt
        s = kp * delta[1] + integ + deriv

    elif left < 30:  # On est proche du mur gauche
        print("Proche du mur gauche")
        delta = [last_delta, 21 - left]
        prop = kp * delta[1]
        integ += ki * delta[1] * dt
        deriv = kd * (delta[1] - delta[0]) / dt
        s = kp * delta[1] + integ + deriv

    elif right < 30:  # On est proche du mur droit
        print("Proche du mur droit")
        delta = [last_delta, right - 21]
        prop = kp * delta[1]
        integ += ki * delta[1] * dt
        deriv = kd * (delta[1] - delta[0]) / dt
        s = borne(prop, 30) + borne(integ, 30) + borne(deriv, 30)

    else:
        s = 0

    mybot.powerboard.set_speed(80 + s, 80 - s)

    Log.add_to_current_data(left=left, right=right)
    Log.add_to_current_data(front=mybot.sonars.read_4_sonars()[0])
    Log.add_to_current_data(stop_dist=stop_dist)
    Log.add_to_current_data(P=prop, I=integ, D=deriv)

    return delta[1], integ


def turn_right(mybot):
    turnSpeed = 80
    
    mag = mybot.imu.read_mag_raw()
    heading = mybot.imu.heading_deg(mag[0], mag[1])

    headingAimed = 0
    if 315 <= heading or heading < 45:
        headingAimed = 270
    elif 45 <= heading < 135:
        headingAimed = 0
        i = 1
        while abs(heading - headingAimed) > 2:
            if 180 < heading <= 360:
                mybot.powerboard.set_speed(-turnSpeed, turnSpeed)
                if i%2 == 0:
                    turnSpeed *= .93
                    i+=1
            else:
                mybot.powerboard.set_speed(turnSpeed, -turnSpeed)
                if i%2 == 1:
                    turnSpeed *= .93
                    i+=1
            mag = mybot.imu.read_mag_raw()
            heading = mybot.imu.heading_deg(mag[0], mag[1])
            print(heading)
    elif 135 <= heading < 225:
        headingAimed = 90
    else:
        headingAimed = 180
    
    i = 1
    while abs(heading - headingAimed) > 2:
        if heading <= headingAimed:
            mybot.powerboard.set_speed(-turnSpeed, turnSpeed)
            if i%2 == 0:
                turnSpeed *= .93
                i+=1
        else:
            mybot.powerboard.set_speed(turnSpeed, -turnSpeed)
            if i%2 == 1:
                turnSpeed *= .93
                i+=1
        mag = mybot.imu.read_mag_raw()
        heading = mybot.imu.heading_deg(mag[0], mag[1])
        print(heading)
        
    mybot.powerboard.set_speed(0, 0)
    time.sleep(1)


def turn_left(mybot):
    turnSpeed = 80

    mag = mybot.imu.read_mag_raw()
    heading = mybot.imu.heading_deg(mag[0], mag[1])

    headingAimed = 0
    if 315 <= heading or heading < 45:
        headingAimed = 90
    elif 45 <= heading < 135:
        headingAimed = 180
    elif 135 <= heading < 225:
        headingAimed = 270
    else:
        headingAimed = 0

        i = 1
        while abs(heading - headingAimed) > 2:
            if 180 < heading <= 360:
                mybot.powerboard.set_speed(-turnSpeed, turnSpeed)
                if i%2 == 0:
                    turnSpeed *= .93
                    i+=1
            else:
                mybot.powerboard.set_speed(turnSpeed, -turnSpeed)
                if i%2 == 1:
                    turnSpeed *= .93
                    i+=1

            mag = mybot.imu.read_mag_raw()
            heading = mybot.imu.heading_deg(mag[0], mag[1])
            print(heading)

    i = 1    
    while abs(heading - headingAimed) > 2:
        if heading <= headingAimed:
            mybot.powerboard.set_speed(-turnSpeed, turnSpeed)
            if i%2 == 0:
                turnSpeed *= .93
                i+=1

        else:
            mybot.powerboard.set_speed(turnSpeed, -turnSpeed)
            if i%2 == 1:
                turnSpeed *= .93
                i+=1
        mag = mybot.imu.read_mag_raw()
        heading = mybot.imu.heading_deg(mag[0], mag[1])
        print(heading)
        
    mybot.powerboard.set_speed(0, 0)
    time.sleep(1)


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
        s = borne(k * (dist - aimed_dist) + offset, 50)
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

    magx_min = -629
    magx_max = 2079
    magy_min = -4321
    magy_max = -1749

    mybot.imu.fast_heading_calibration (magx_min, magx_max, magy_min, magy_max)

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
            integ = 0

            if mybot.sonars.read_left() > mybot.sonars.read_right():
                print("turn left")
                turn_left(mybot)
                time.sleep(0.42)

            else:
                print("turn right")
                turn_right(mybot)
                time.sleep(0.42)

        t1_loop = time.time()
        dt_loop = t1_loop - t0_loop
        dt_sleep = dt - dt_loop
        if dt_sleep > 0:
            time.sleep(dt_sleep)

    mybot.powerboard.set_speed(0, 0)
    mybot.end()

