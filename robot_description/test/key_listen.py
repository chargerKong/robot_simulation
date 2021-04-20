#coding: utf-8
import os
import sys
import tty, termios
import time

if __name__ == '__main__':
    print("Reading form keybord")

    print(   """   i
j  k  l
   m""")

    print('press Q to quit')
    while True:
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        # print 'error'
        if ch == 'i':
            print(     'move forward')

        elif ch == 'm':
            print('move back')

        elif ch == 'j':
            print("turn left!")

        elif ch == 'l':
            print("turn right!")

        elif ch == 'u':
            print("turn right!")

        elif ch == 'o':
            print("turn right!")

        elif ch == 'k':
            print("stop motor!")

        elif ch == 'q':
            print("shutdown!")

        elif ord(ch) == 25:
            print("left arrow")

            break
        elif ord(ch) == 0x3:
            # 这个是ctrl c
            print("shutdown")

            break
        print((ch), ch.__class__)
#         print("Reading form keybord")
#
#         print(        """   i
# j  k  l
#    m""")
#
#         print(       'press Q or ctrl+c to quit')


