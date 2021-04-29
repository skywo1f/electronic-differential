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
altitude = 0
thetaCoord = 0
phiCoord = 0
rollCoord = 0

latitude2 = 0
longitude2 = 0
altitude2 = 0
thetaCoord2 = 0
phiCoord2 = 0
rollCoord2 = 0



message =b"0 0 0 0"             
encoding = 'utf-8'

posData = b"0 0 0 0 0 0 0"
posData2 = b"0 0 0 0 0 0 0"

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 8190        # Port to listen on (non-privileged ports are > 1023)

PORT2 = 8191        #port for follower vehicle

'''
def quaternionToYaw(qz,qw):
    Yaw=180*m.atan2(2*(qw*qz),1-2*qz*qz)/m.pi
    return Yaw
'''

def quaternionToYaw(qx,qy,qz,qw):
    p1 = 2*(qw*qz + qx*qy)
    p2 = 1 - 2*(qy*qy + qz*qz)
    Yaw = 180*m.atan2(p1,p2)/m.pi
    return Yaw

def quaternionToPitch(qx,qy,qz,qw):
    sinp = 2*(qw*qy - qz*qx)
    Pitch = 180*m.asin(sinp)/m.pi
    return Pitch

def quaternionToRoll(qx,qy,qz,qw):
    p1 = 2*(qw*qx + qy*qz)
    p2 = 1 - 2*(qx*qx + qy*qy)
    Roll = 180*m.atan2(p1,p2)/m.pi
    return Roll

def parseMsg(received):
    individuals = received.split(b' ')
#    print("individual is ")
#    print(individuals)
    return individuals

def thread_talk_to_driver():
    global message
    message =b"0 0 0 0"
#    print("connecting to main thread")
    socketZmq = context.socket(zmq.REQ)
    socketZmq.connect("tcp://localhost:5584")
    firstLetter = b"0 0 0 0 0 0 0 0 0 0 0 0"                # lat long alt theta phi roll
    socketZmq.send(firstLetter)
    received = socketZmq.recv()
    message = parseMsg(received)
#    print(type(message[0]))
    while True:
        sendMsg = str(latitude).encode() + b" " + str(longitude).encode() + b" " + str(altitude).encode() + b" " + str(thetaCoord).encode() + b" " + str(phiCoord).encode() + b" " + str(rollCoord).encode() + b" " + str(latitude2).encode() + b" " + str(longitude2).encode() + b" " + str(altitude2).encode() + b" " + str(thetaCoord2).encode() + b" " + str(phiCoord2).encode() + b" " + str(rollCoord2).encode()
#        print(sendMsg)
        socketZmq.send(sendMsg)
        received = socketZmq.recv()
        message = parseMsg(received)
#        print(rollCoord)
#        print(thetaCoord)

def thread_talk_to_gazebo(PORT):
        global posData
        posdata = b"0 0 0 0 0 0 0"                                  # x y z qx qy qz qw
        global posData2
        posdata2 = b"0 0 0 0 0 0 0"                                  # x y z qx qy qz qw

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen()
        conn, addr = s.accept()
        with conn:
            while True:
                if PORT == 8190:
                    posData = conn.recv(1024)
                    if not posData:
                        break
                    #print(message)
                    thisMsg = b' '.join([message[0],message[1]])
                    conn.sendall(thisMsg)
                if PORT == 8191:
                    posData2 = conn.recv(1024)
                    if not posData2:
                        break
                    thisMsg = b' '.join([message[2],message[3]])
                    conn.sendall(thisMsg)

#                print(posData)
#                print("received ")

def thread_convert_to_coord():
    global latitude
    latitude = 0
    global longitude
    longitude = 0
    global altitude
    altitude = 0
    global thetaCoord
    thetaCoord = 0
    global phiCoord
    phiCoord = 0
    global rollCoord
    rollCoord = 0
    while(True):
        alphabet = posData.decode(encoding)
        positions = re.split(' ',alphabet)
        x = float(positions[0])
        y = float(positions[1])
        z = float(positions[2])
        qx = float(positions[3])
        qy = float(positions[4]) 
        qz = float(positions[5])  #quaternion z
        qw = float(positions[6])  #quaternion w

        latitude = x
        longitude = z
        altitude = y
        thetaCoord = quaternionToYaw(qx,qy,qz,qw)%360
        phiCoord = quaternionToPitch(qx,qy,qz,qw)
        rollCoord = quaternionToRoll(qx,qy,qz,qw)

def thread_convert_to_coord2():
    global latitude2
    latitude2 = 0
    global longitude2
    longitude2 = 0
    global altitude2
    altitude2 = 0
    global thetaCoord2
    thetaCoord2 = 0
    global phiCoord2
    phiCoord2 = 0
    global rollCoord2
    rollCoord2 = 0
    while(True):
        alphabet = posData2.decode(encoding)
        positions = re.split(' ',alphabet)
#        print(positions)
        x = float(positions[0])
        y = float(positions[1])
        z = float(positions[2]) 
        qx = float(positions[3])
        qy = float(positions[4])
        qz = float(positions[5])  #quaternion z
        qw = float(positions[6])  #quaternion w

        latitude2 = x
        longitude2 = z
        altitude2 = y
        thetaCoord2 = quaternionToYaw(qx,qy,qz,qw)%360
        phiCoord2 = quaternionToPitch(qx,qy,qz,qw)
        rollCoord2 = quaternionToRoll(qx,qy,qz,qw)


if __name__ == "__main__":
    firstThread = threading.Thread(target=thread_talk_to_driver)
    firstThread.start()

    secondThread = threading.Thread(target=thread_talk_to_gazebo,args=(PORT,))
    secondThread.start()

    fifthThread = threading.Thread(target=thread_talk_to_gazebo,args=(PORT2,))
    fifthThread.start()

    thirdThread = threading.Thread(target=thread_convert_to_coord)
    thirdThread.start()

    fourthThread = threading.Thread(target=thread_convert_to_coord2)
    fourthThread.start()
 
