# This file contains the variable constants to be used in the main program.
# also contains various library usage

import requests, heapq, time, json
from pprint import pprint
from NodeList import *
from FloorPlanManager import *

#http://showmyway.comp.nus.edu.sg/getMapInfoGraphView.php?Building=AAA&Level=BBB
url = "http://ShowMyWay.comp.nus.edu.sg/getMapInfo.php"

stepLength = 50 #cm #might no longer be needed

# Allowable distance from target destination to be considered as reached
offsetDistance = 50