#!/usr/bin/env python3

import socket
import curses
import threading

global speedOpt
HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 8090        # Port to listen on (non-privileged ports are > 1023)
def draw_menu(stdscr):
    k = 0
    global speedOpt
    speedOpt = 5
    while (k != ord('q')):
        stdscr.refresh()
        k = stdscr.getch()
        if k == curses.KEY_DOWN:
            speedOpt = 4
        elif k == curses.KEY_UP:
            speedOpt = 2
        elif k == curses.KEY_RIGHT:
            speedOpt = 3
        elif k == curses.KEY_LEFT:
            speedOpt = 1
#        print(speedOpt)


def thread_read_keys():
    curses.wrapper(draw_menu)

def thread_talk_to_gazebo():
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            while True:
                data = conn.recv(1024)
#                print("received ")
#                print(data)
                if not data:
                    break
                conn.sendall(str(speedOpt).encode('utf-8'))


if __name__ == "__main__":
    firstThread = threading.Thread(target=thread_read_keys)
    firstThread.start()

    secondThread = threading.Thread(target=thread_talk_to_gazebo)
    secondThread.start()

