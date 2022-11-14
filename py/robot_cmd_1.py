import dartv2b
import time
if __name__ == "__main__":

    mybot = dartv2b.DartV2()
    right, left = mybot.get_front_encoders()
    a = time.time()
    while mybot.get_front_encoders()[0] - right < 300:
        mybot.set_speed(100,-100)
        t = time.time() - a
        print(f'Le temps requis est {t} secondes')
    mybot.set_speed(0, 0)

    mybot.end()