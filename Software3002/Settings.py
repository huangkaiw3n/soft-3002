# This file contains the variables and constants to be used in the main program.
# also contains various library usage

import requests, heapq, time, json
from pprint import pprint
from NodeList import *
from FloorPlanManager import *
from WiFi import internet_on
from Espeak import *

#http://showmyway.comp.nus.edu.sg/getMapInfoGraphView.php?Building=AAA&Level=BBB
url = "http://ShowMyWay.comp.nus.edu.sg/getMapInfo.php"

stepLength = 50 #cm #might no longer be needed

# Allowable distance from target destination to be considered as reached
offsetDistance = 50
# Allowable difference in heading to considered as acceptable
offSetAngle = 5.0

#-----------------------------------------------------------------------------
# These values will be updated real-time by the arduino through UART when 
# navigation begins
currentX = 0
currentY = 0
currentHeading = 0
#-----------------------------------------------------------------------------
# String to hold data from uart
inStr = "" 