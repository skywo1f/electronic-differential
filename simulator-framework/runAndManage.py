import subprocess
import keyboard
import time
import sys
import os
import signal
'''
this class makes it easier to run jobs.
Example:
runner = jobs()
gazeboBuildDir = "/home/iviti/gazebo-simulator/build/"
gaz = ["gazebo", "--verbose", gazeboBuildDir + "../velodyne.world"]
runner.launchJob(gaz)
time.sleep(5)
runner.endThis()
'''


class jobs:
    def __init__(self):
        self.jobList = []
        self.jobNum = 0

    def launchJob(self, jobAndArgs):
        p = subprocess.Popen(jobAndArgs)
        self.jobList.append(p)
        self.jobNum = self.jobNum + 1
        return self.jobNum - 1

    def shutDownAllJobs(self):
        for i in range(self.jobNum):
            print(self.jobList[i].pid)
            self.jobList[i].terminate()
#            self.jobList[i].kill()
            self.jobList[i].wait()
#            os.kill(self.jobList[i].pid,0)
#            time.sleep(0.1)
    def endThis(self):
        self.shutDownAllJobs()
        os.system("killall -9 gzserver")
        os.system("killall -9 gzclient")
        time.sleep(1)
        pid = os.getpid()
        print("shutting down")
#        os.kill(pid, signal.SIGTERM)
        os._exit(0)


    def endJob(self,num):
        self.jobList[num].terminate()




