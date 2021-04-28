from twoStepDriving import *
from runAndManage import *
import time

if __name__ == "__main__":
    thisPath = os.path.abspath(os.getcwd())
    runner = jobs()
    d2r = ["python3",thisPath +  "/../gazebo-simulator/build/driver-to-gazebo.py"]                                                   #driver to ros communication
    r2g = [thisPath + "/../gazebo-simulator/build/vel","4"]                                                                        #ros to gazebo communication
    r2g2 = [thisPath + "/../gazebo-simulator/build/fol","4"]                                                                        #ros to gazebo communication
    gaz = ["gazebo", "--verbose", thisPath + "/../gazebo-simulator/velodyne.world"]                                         #start gazebo
    runner.launchJob(d2r)
    path = np.loadtxt('coords.txt')
    sp11 = twoStepDriving(path,"sim")
    runner.launchJob(gaz)
    time.sleep(3)
    runner.launchJob(r2g)
    runner.launchJob(r2g2)
    going = True
    try:
        while going:
#        while not sp11.returnSimDone():
            if sp11.numWp_g + 1 == len(sp11.path):
                going = False
            time.sleep(1)

        runner.endThis()
#        runner.shutDownAllJobs()
        exit()  
        sys.exit()  
    except:
        runner.endThis()
        exit()

