class GetKey:
    def __init__(self):
        import tty, sys, termios # import termios now or else you'll get the Unix version on the Mac

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

if __name__ == "__main__":
    getkey = GetKey()

    while True:
        ch = getkey()
        ich = ord(ch)
        if ich>=32:
            print (ch,ich,"%2.2X"%(ich))
        else:
            print (ich,"%2.2X"%(ich))
        if ich == 27 or ich==3:   # 3 = CTRL+C
            break
