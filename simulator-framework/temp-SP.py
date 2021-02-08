




#	print(delta)



'''
def pidServer1():                                                                         #send desired wheel speeds to pid
#connect to driver-to-pid.py interface
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5557")
    

    while True:
        response = str(leftWheelPwr_g)
        message = socket.recv()
        socket.send(response.encode(encoding))

def pidServer2():                                                                         #send desired wheel speeds to pid
#second wheel
    context2 = zmq.Context()
    socket2 = context2.socket(zmq.REP)
    socket2.bind("tcp://*:5558")

    while True:
        response = str(rightWheelPwr_g)
        message = socket2.recv()
        socket2.send(response.encode(encoding))
'''







def driveLoop(path,finalLat,finalLong,firstRun):                                             #starts driving/locator threads and checks for completion
    
    if (firstRun == True):                                                              #on first run, activate the connection to the locator

        '''
        pidDriver1 = threading.Thread(target=pidServer1)  
        pidDriver1.daemon = True
        pidDriver1.start()

        pidDriver2 = threading.Thread(target=pidServer2)  
        pidDriver2.daemon = True
        pidDriver2.start()
        '''
        bothDriver = threading.Thread(target=bothPid)
        bothDriver.daemon = True
        bothDriver.start()

        pointsChecker = threading.Thread(target=numWpUpdate,args=(path, ))
        pointsChecker.daemon = True
        pointsChecker.start()

        gpsChecker = threading.Thread(target=getGPS)
        gpsChecker.daemon = True
        gpsChecker.start()

        gyroChecker = threading.Thread(target=getCompass)
        gyroChecker.daemon = True
        gyroChecker.start()

        imuChecker = threading.Thread(target=getImu)
        imuChecker.daemon = True
        imuChecker.start()

        driveChecker = threading.Thread(target=driver,args=(path,))      
        driveChecker.daemon = True
        driveChecker.start()

        gpsBearingChecker = threading.Thread(target=calcGpsHeading)
        gpsBearingChecker.daemon = True
        gpsBearingChecker.start()

        kalmanRunner = threading.Thread(target=kalman_angle)
        kalmanRunner.daemon = True
        kalmanRunner.start()

        simToCode = simulatedThrow("simulator",5580,5581,5582,5583,5584)                #initialize bridging algorithm for simulator
        simToCode.grabSpeeds()                                                      #grab speeds from driver and prepare to hand to simulator
        simToCode.sendValues()                                                      #take values from simulator and give them to driver
        simToCode.grabValues()                                                      #grab location data from simulator and prepare to give them to driver
        simToCode.convertValues()                                                   #convert from simulated coordinates to gps coordinates
    while True:                                                          
        time.sleep(5)







#driveLoop(path[0],path[1],True)    #actual first step
driveLoop(path,0,0,True)    #for display purposes



