from Node import *

class NodeList(object):
    def __init__(self):
        self.numNodes = 0
        self.NodeList = {}
        
    def __contains__(self, node):
        return node in self.NodeList
    
    def addNode(self, node):
        self.numNodes = self.numNodes + 1
        self.NodeList[node.id] = node
         
    def getNodeById (self, id):
        return self.NodeList[id]
    
    def getNodeByName(self, name):
        for node_i in self.NodeList:
            if node_i.name == name:
                return node_i
    
    def getNumNodes(self):
        return self.numNodes
    

#-----------------------------------------------------------------------------
# LocationNodeList class stores all LocationNodes information including north
#-----------------------------------------------------------------------------   
class LocationNodeList(NodeList):
    def __init__(self, mapData, northInfo):
        NodeList.__init__(self)
        self.north = int(northInfo["northAt"]) #store north bearing
        
        for i in range(0, len(mapData)):  #adds all existing nodes to node list
            myNode = LocationNode(mapData[i])
            self.addNode(myNode)
        
        
#-----------------------------------------------------------------------------
# WifiNodeList class stores all WifiNodes information
#-----------------------------------------------------------------------------  
class WifiNodeList(NodeList):
    def __init__(self, mapData):
        NodeList.__init__(self)
        
        for i in range(0, len(mapData)):  #adds all existing nodes to node list
            myNode = WifiNode(mapData[i])
            self.addNode(myNode)
            
    def getNodeByMacAddr(self, macAddr):
        for node_i in self.NodeList:
            if node_i.macAddr == macAddr:
                return node_i 