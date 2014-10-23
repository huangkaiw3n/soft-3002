# This module contains:
# - NodeList template and its inheritance: LocationNodeList and WifiNodeList
# - AdjacncyList class to store LocationNode

from Node import *

#-----------------------------------------------------------------------------
# NodeList class template for LocationNodeList class and WifiNodeList class
#-----------------------------------------------------------------------------
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
            
  
#-----------------------------------------------------------------------------
# AdjList class store the map information
# only includes LocationNodes
#----------------------------------------------------------------------------- 
class AdjList(object):
    def __init__(self, nlist):
        self.graph = {}
        
        for i in range(1, nlist.getNumNodes()+1):
            tempNode1 = nlist.getNodeById(i)
            adj = {}
            for j in range(0, tempNode1.getNumNbr()):
                tempNode2 = nlist.getNodeById(tempNode1.getNbrId(j))
                adj[tempNode1.getNbrId(j)] = tempNode1.distanceTo(tempNode2)
        
            self.graph[i] = adj

    def getGraph(self):
        return self.graph
