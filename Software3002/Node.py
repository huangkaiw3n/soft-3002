# This module contains template for Node class and its inheritance: LocationNode and WifiNode

import math

#-----------------------------------------------------------------------------
# Node class template for LocationNode class and WifiNode class
#-----------------------------------------------------------------------------
class Node(object):
    def __init__(self, nodeInfo):
        self.name = nodeInfo["nodeName"]
        self.id = int(nodeInfo["nodeId"])
        self.x = int(nodeInfo["x"])
        self.y = int(nodeInfo["y"])
        
    def distanceTo(self, nbr):
        xDistance = math.fabs(self.x - nbr.x)
        yDistance = math.fabs(self.y - nbr.y)
        return math.sqrt(xDistance ** 2 + yDistance ** 2)
    
        
#-----------------------------------------------------------------------------
# LocationNode class inherits from Node class
# store information about each location and neighboring locations
#-----------------------------------------------------------------------------
class LocationNode(Node):
    def __init__(self, nodeInfo):
        Node.__init__(self, nodeInfo)
        self.addNeighbor(nodeInfo)
        self.linkTo = []      #list of connected node ids coverted to integer
        self.neighbourAngle = nodeInfo["angle"] # "angle":{"1":"x","3":"x","4":"x"}
        for i in range(0, len(self.neighborsList)):
            self.linkTo.append(int(self.neighborsList[i]))

    def getNumNbr(self):
        return len(self.linkTo)
    
    def getNbrId(self, num):
        return self.linkTo[num]
                 
    def addNeighbor(self, nodeInfo):
        self.neighborsList = nodeInfo["linkTo"].split(",") #list of strings, must convert to integer first

#-----------------------------------------------------------------------------
# WifiNode class inherits from Node class
# store MAC address of the wifi router
#-----------------------------------------------------------------------------  
class WifiNode(Node):
    def __init__(self, nodeInfo):
        Node.__init__(self, nodeInfo)
        self.macAddr = nodeInfo["macAddr"]

