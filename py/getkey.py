class GetKey:
    def __init__(self):
        import platform
        self.system = "Linux"
        if platform.system() == "Windows":
            self.system = "Windows"
        if self.system == "Windows":
            import msvcrt  # no termios neither tty on Windows10 , use msvcrt instead
        elif self.system == "Linux":
            import tty, sys, termios # import termios and sys not tesetd on MacOs
    def __call__(self):
        if self.system == "Windows":
            import msvcrt  
            ch=chr(0)
            if msvcrt.kbhit():
                ch = msvcrt.getch()
        if self.system == "Linux":
            import tty, sys, termios 
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

if __name__ == "__main__":
    import time
    getkey = GetKey()

    while True:
        ch = getkey()
        ich = ord(ch)
        if ich>=32:
            print (ch,ich,"%2.2X"%(ich))
        elif ich>0:
            print (ich,"%2.2X"%(ich))
        if ich == 27 or ich==3:   # 3 = CTRL+C
            break
        time.sleep(0.05)