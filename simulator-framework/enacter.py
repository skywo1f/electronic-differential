from multiprocessing import Process, Value, Array
import multiprocessing
import numpy as np
import time
import sys
import signal
import serial
import re
import zmq
encoding = 'utf-8'
import crcmod
import binascii



#todo: combine messages (init,rnck, etc... into one subclass)

class enact:
    def __init__(self):
        try:
            self.ser = serial.Serial(            
                port='/dev/ttyS0',
                baudrate = 115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0
            )
        except:
            print("could not find serial 0 channel")

    def runSpeedRead(self):        
        currentV1 = multiprocessing.Manager().list([0])                         #current value 1 (left wheel speed)
        currentV2 = multiprocessing.Manager().list([0])                         #current value 2 (right wheel speed)
        lastT_g = multiprocessing.Manager().list([0])
        processes = []
        currentV1[0] = 0
        currentV2[0] = 0
        lastT_g[0] = 0

        pL = Process(target=self.readSP,args=(currentV1,"left"))                     #constantly read left wheel change commands
        processes.append(pL)
        pR = Process(target=self.readSP,args=(currentV2,"right"))                    #constantly read right wheel change commands
        processes.append(pR)
        pW = Process(target=self.writeWheel,args=(currentV1,currentV2,lastT_g)) #apply wheel change commands
        processes.append(pW)
        
        self.writeInit()
        [x.start() for x in processes]
        try:
            while True:
                time.sleep(5)
        except:
            exit()

    def readSP(self,currentV,whichWheel):
        currentV[0] = 0
        lastE = 0
        sumE = 0
        currentE = 0
        changeV = 0
        if(whichWheel == "right"):
            context = zmq.Context()
            socketZmq = context.socket(zmq.REQ)
            socketZmq.connect("tcp://localhost:5557")
        if(whichWheel == "left"):
            context = zmq.Context()
            socketZmq = context.socket(zmq.REQ)
            socketZmq.connect("tcp://localhost:5558")
        message = b"0"
        socketZmq.send(b"request")
        message = socketZmq.recv()
        desiredVal = float(message.decode(encoding))
        currentV[0] = 0
        turnRate = 10
        lastTime = time.time()
        smoothTics = 0
        lastVal = currentV[0]
        ramp = 0.1                                      #larger values mean faster ramp
        while(True):
            socketZmq.send(b"request")
            message = socketZmq.recv()
            requested = float(message.decode(encoding))
            if requested < 0.1 and requested > -0.1:
                currentV[0] = 0
            else:
                currentV[0] = requested*(ramp) + lastVal*(1-ramp)
            lastVal = currentV[0]
            time.sleep(0.001)




    def writeWheel(self,currentV1,currentV2,lastT_g):
        cap = 0.6
        maxPwr = 150
        #one fifth max power
        #cap = 0.5                                                                      #one half max power                                                             #rpm's to digital
        while True:
            rescaled = float(currentV1[0])
            rescaled2 = float(currentV2[0])
#               print(rescaled,rescaled2)
            if rescaled > maxPwr*cap:
                rescaled = maxPwr*cap
            if rescaled < -maxPwr*cap:
                rescaled = -maxPwr*cap
            if rescaled2 > maxPwr*cap:
                rescaled2 = maxPwr*cap
            if rescaled2 < -maxPwr*cap:
                rescaled2 = -maxPwr*cap
            print(rescaled,rescaled2)
            self.writeSpeed(rescaled,rescaled2)          #set wheel speeds
            self.watchDog(lastT_g)
            time.sleep(0.001)

    
        
    def writeSpeed(self,wsl,wsr):
        head = 'wrsd'
        daeh = 'dsrw'
        bh = head.encode('utf-8')       #head in bytes form
        hb = daeh.encode('utf-8')
        wsl00 = int(round(wsl,2)*100)
        wsr00 = int(round(wsr,2)*100)
        wsl00s = wsl00 & 0xFF
        wsl00b = (wsl00 & 0xFF00)>>8
        wsr00s = wsr00 & 0xFF
        wsr00b = (wsr00 & 0xFF00)>>8
        ydobi = (wsr00b<<24) | (wsr00s<<16) | (wsl00b<<8) | (wsl00s)
        bodyi = (wsl00s<<24) | (wsl00b<<16) | (wsr00s<<8) | (wsr00b)
        crc32_func = crcmod.mkCrcFun(0x104c11db7, initCrc=0xFFFFFFFF,rev=False)
        crcOut = crc32_func(hb)
        crc32_func = crcmod.mkCrcFun(0x104c11db7, initCrc=crcOut,rev=False)
        crc2 = crc32_func(bodyi.to_bytes(4,'little'))
        self.ser.write(bh)
        self.ser.write(ydobi.to_bytes(4,'little'))
        self.ser.write(crc2.to_bytes(4,'little'))
        time.sleep(0.001)

#send watchdog message        
    def watchDog(self,lastT_g):
        head = 'rnck'                                                                   #initialize header
        daeh = 'kcnr'                                                                   #reverse init
        bh = head.encode('utf-8')                                                       #string to byte
        hb = daeh.encode('utf-8')
        body = '0000'                                                                   #generic body message (doesnt matter)
        bb = body.encode('utf-8')                                                       #string to bytes the body
        crc32_func = crcmod.mkCrcFun(0x104c11db7, initCrc=0xFFFFFFFF,rev=False)         #generate a crc machine with that poly
        crcOut = crc32_func(hb)                                                         #run head through crc machine
        crc32_func = crcmod.mkCrcFun(0x104c11db7, initCrc=crcOut,rev=False)             #generate another crc machine, but with the last crc as initializer
        crc2 = crc32_func(bb)                                                           #run body through that crc
        hcrc2 = hex(crc2)                                                               #convert output int to hex
        bhcrc2 = bytearray.fromhex(hcrc2[2:])                                           #convert hex to byte (and remove 0x)
        rbhcrc2 = bhcrc2[::-1]                                                          #reverse
        msg = bh + bb + rbhcrc2                                                         #combine message
        self.ser.write(msg)


#initialize st-can system
    def writeInit(self):
        head = 'init'                                                                   #initialize header
        daeh = 'tini'                                                                   #reverse init
        bh = head.encode('utf-8')                                                       #string to byte
        hb = daeh.encode('utf-8')
        body = '0000'                                                                   #generic body message (doesnt matter)
        bb = body.encode('utf-8')                                                       #string to bytes the body
        crc32_func = crcmod.mkCrcFun(0x104c11db7, initCrc=0xFFFFFFFF,rev=False)         #generate a crc machine with that poly
        crcOut = crc32_func(hb)                                                         #run head through crc machine
        crc32_func = crcmod.mkCrcFun(0x104c11db7, initCrc=crcOut,rev=False)             #generate another crc machine, but with the last crc as initializer
        crc2 = crc32_func(bb)                                                           #run body through that crc
        hcrc2 = hex(crc2)                                                               #convert output int to hex
        bhcrc2 = bytearray.fromhex(hcrc2[2:])                                           #convert hex to byte (and remove 0x)
        rbhcrc2 = bhcrc2[::-1]                                                          #reverse
        msg = bh + bb + rbhcrc2                                                         #combine message
        self.ser.write(msg)                                                                  #write message


