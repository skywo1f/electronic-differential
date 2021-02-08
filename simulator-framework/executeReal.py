from twoStepDriving import *
from runAndManage import *
import time

if __name__ == "__main__":

    path = np.loadtxt('coords.txt')
    sp11 = twoStepDriving("real")
    try:
        while True:
            time.sleep(5)
    except:
        exit()
