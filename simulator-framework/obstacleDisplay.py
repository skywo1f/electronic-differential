import time
import zmq
import threading
import numpy as np
import re
import serial
from tkinter import *
import math
import pyrealsense2 as rs
from lightSimple import *
from obstacleTracking import *




if __name__ == "__main__":

	obs = obstacleTracking()
	if obs.getSonar.values.any() < 50:
		print("sonar sees something")
