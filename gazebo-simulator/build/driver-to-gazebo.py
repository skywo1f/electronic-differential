#!/usr/bin/env python3

import socket
import curses
import threading
import numpy as np

import re
import math as m
import threading
import zmq
context = zmq.Context()

latitude = 0
longitude = 0
thetaCoord = 0
global message
message =b"0 0"             #wheel strengths pass through unparsed
encoding = 'utf-8'

global posData
posData = b"0 0 0 0"
global speedOpt
HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 8090        # Port to listen on (non-privileged ports are > 1023)


def quaternionToYaw(qz,qw):
    Yaw=180*m.atan2(2*(qw*qz),1-2*qz*qz)/m.pi
    return Yaw

def thread_talk_to_driver():
    global message
    message =b"0 0"
    print("connecting to main thread")
    socketZmq = context.socket(zmq.REQ)
    socketZmq.connect("tcp://localhost:5584")
    firstLetter = b"0 0 0"
    socketZmq.send(firstLetter)
    message = socketZmq.recv()
    print("connected")
    while True:
        socketZmq.send(str(latitude).encode() + b" " + str(longitude).encode() + b" " + str(thetaCoord).encode())
        message = socketZmq.recv()
#        print(message)
#        print(thetaCoord)

def thread_talk_to_gazebo():
        global posData
        posdata = b"0 0 0 0"
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            while True:
                posData = conn.recv(1024)
#                print("received ")
#                print(posData)
                if not posData:
                    break
                conn.sendall(message)

def thread_convert_to_coord():
    global latitude
    latitude = 0
    global longitude
    longitude = 0
    global thetaCoord
    thetaCoord = 0
    while(True):
        alphabet = posData.decode(encoding)
        positions = re.split(' ',alphabet)
#        print(positions)
        x = float(positions[0])
        z = float(positions[1])
        qz = float(positions[2])  #quaternion x
        qw = float(positions[3])  #quaternion w

        latitude = x
        longitude = z
        thetaCoord = quaternionToYaw(qz,qw)%360

if __name__ == "__main__":
    firstThread = threading.Thread(target=thread_talk_to_driver)
    firstThread.start()

    secondThread = threading.Thread(target=thread_talk_to_gazebo)
    secondThread.start()

    thirdThread = threading.Thread(target=thread_convert_to_coord)
    thirdThread.start()
    