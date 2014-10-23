# This module contains FloorPlan and FloorPlanList used to store different maps


#-----------------------------------------------------------------------------
# FloorPlan class store the raw JSON data of a map of a level floor plan
#-----------------------------------------------------------------------------  
class FloorPlan(object):
    def __init__(self, mapID, jsonData):
        self.mapID = mapID
        self.buildingName = mapID[0]
        self.level = mapID[1]
        self.jsonData = jsonData
        

#-----------------------------------------------------------------------------
# FloorPlanList class stores all available FloorPlans in a list
#-----------------------------------------------------------------------------  
class FloorPlanList(object):
    def __init__(self):
        self.list = {}
        self.numFloorPlans = 0
        
    def addFloorPlan(self, newFloorPlan):
        #store floor plan in a dictionary with key is obtained from getKey
        key = self.getKey(newFloorPlan.buildingName, newFloorPlan.level)
        self.list[key] = newFloorPlan.jsonData
        self.numFloorPlans = self.numFloorPlans + 1
    
    def isFloorPlanAvailable(self, FloorPlan):
        key = self.getKey(FloorPlan.buildingName, FloorPlan.level)
        return list.has_key(key)
    
    #convert buildingName + level to key. For eg: buildingName = COM1, level 1 ==> key = "COM1 1"
    def getKey(self, buildingName, level):
        return buildingName + " " + str(level)
    
    