# main module of the system

from Settings import *

# rPi GPIO interrupt will activate UART read on rPi's side
# flag to tell UART handler whether we are taking in currentXYheading during
# navigation or we are taking in keyad values when outside navigation
navigation = 0
#-----------------------------------------------------------------------------
# These values will be updated real-time by the arduino through UART when 
# navigation begins
currentX = 0
currentY = 0
currentHeading = 0
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# Sample command to play Intro.txt audio
# espeak -v+f3 -s100 -f /home/pi/Vision/Intro.txt --stdout | aplay
# all audio will be put in .txt files to resolve a lag issue with audio playback
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# initialise building name and level
# get JSON response from the url and parse to data
#-----------------------------------------------------------------------------
def initialise():
    control = int(input("Press 1 to input new map, Press 2 to load Com1 Level 2\n"))

    if control == 1:
        buildingName = str(raw_input("Enter building name\n"))
        level = int(raw_input("Enter level\n"))
        #buildingName = "oksure"
        #level = 1337

        mapID = [buildingName, level]
        params = dict(Building = buildingName, Level = level)
        response = requests.get(url=url, params = params)
        data = response.json()

    elif control == 2:
        json_data=open('data.txt')
        data = json.load(json_data)
        json_data.close()
    
    #FloorPlanDatabase = FloorPlanList()  #initialise database
    #myFloorPlan = FloorPlan(mapID, data)    
    #FloorPlanDatabase.addFloorPlan(myFloorPlan) #add floor plan to database
    
    return data


#-----------------------------------------------------------------------------
# startup function will play audio data when the system first starts up
#-----------------------------------------------------------------------------
def startUp():
    # to audio: 1. Welcome Message. Press star hash at anytime to repeat last message
    # to audio: 2. Please select the current map. (optional)
    # to audio: 3. Press 1 to begin navigation. Press 2 to select another map
    # to audio: 4. Please enter your current node ID
    # to audio: 5. Please enter your destination node ID
    return


#-----------------------------------------------------------------------------
# main
#-----------------------------------------------------------------------------
def main():
    data = initialise()
    pprint(data)
    global currentX
    global currentY
    global currentHeading

    nodeList = LocationNodeList(data["map"], data["info"])
    wifiList = WifiNodeList(data["wifi"])
    
    aList = AdjList(nodeList)        #constructs adjlist from node list

    currentInstruction = int(input("Press 1 to begin navigating"))
    
    if (currentInstruction == 1):
        startNode = nodeList.getNodeById(int(input("Input start node id")))
        destinationNode = nodeList.getNodeById(int(input("Input destination node id")))
        
        cost, path = shortestPath(aList.getGraph(), startNode.id, destinationNode.id)
        
        currentX = startNode.x
        currentY = startNode.y
        i = 1

        while (isReached(currentX, currentY, destinationNode.x, destinationNode.y) == False):
            
            targetNode = nodeList.getNodeById(path[i])
            
            while (isReached(currentX, currentY, targetNode.x, targetNode.y) == False):
  
                time.sleep(1) #assume 1 step 1 second
               
                direction, degree  = computeDirection(currentHeading, currentX, currentY, targetNode.x, targetNode.y, nodeList.north)
                print (direction, degree)

                #just for testing. to be updated through Uart by arduino
                currentX = int(input("Input currentX\n"))
                currentY = int(input("Input currentY\n"))
                currentHeading = int(input("Input currentHeading\n"))
                #currentX, currentY = updateCurrentLocation(currentX, currentY, currentHeading, 1)
            
            #output to audio here: 
            #"Reached Node path[i]"

            if targetNode != destinationNode : #prevent overflow
                i = i + 1 
    else:
        print ("test") 


#-----------------------------------------------------------------------------
# check whether the destination is reached.
# return True if current location is within 0.5 meter radius of destination
#-----------------------------------------------------------------------------
def isReached(currentX, currentY, nextX, nextY):
    yDistance = math.fabs((nextY * 1.0 - currentY * 1.0) * 1.0)
    xDistance = math.fabs((nextX * 1.0 - currentX * 1.0) * 1.0)
    
    d = math.sqrt(yDistance ** 2 + xDistance ** 2)
    
    if (d < offsetDistance):
        return True
    else:
        return False
     
     
#-----------------------------------------------------------------------------
# compute direction based on current coordinate and heading,
# since currentHeading is with respect to the map's North, it will be offset
# to become with respect to the absolute north.
#-----------------------------------------------------------------------------    
def computeDirection(currentHeading, currentX, currentY, nextX, nextY, north):
    currentHeadingWithOffset = currentHeading + north
    if (currentHeadingWithOffset >= 360.0):
        currentHeadingWithOffset = currentHeadingWithOffset - 360.0

    yDistance = (nextY * 1.0 - currentY * 1.0) * 1.0
    xDistance = (nextX * 1.0 - currentX * 1.0) * 1.0
    
    if (yDistance == 0):
        if (nextX * 1.0 - currentX * 1.0) > 0.0:
            targetHeading = 90.0
        else:
            targetHeading = 270.0
    elif (xDistance == 0):
        if (nextY * 1.0 - currentY * 1.0) > 0.0:
            targetHeading = 0.0
        else:
            targetHeading = 180.0
    else:
        targetHeading = math.degrees(math.atan(xDistance/yDistance))
        
        if(xDistance < 0.0 and yDistance > 0.0): # second quadrant rectification
            targetHeading = 360.0 + targetHeading
        elif(xDistance < 0.0 and yDistance < 0.0): # third quadrant
            targetHeading = targetHeading + 180.0
        elif(xDistance > 0.0 and yDistance < 0.0): # fourth quadrant
            targetHeading = targetHeading + 180.0
    #--------------------------------------------------------------------------
    diffAngle = currentHeadingWithOffset - targetHeading
    
    if (diffAngle < -180.0):
        return "turn left", 360.0-math.fabs(diffAngle)
    elif (-180.0 < diffAngle and diffAngle < -5.0):
        return "turn right", math.fabs(diffAngle)
    elif (-5.0 < diffAngle and diffAngle < 5.0):
        return "straight", 0.0
    elif (5.0 < diffAngle and diffAngle < 180.0):
        return "turn left", diffAngle
    elif (diffAngle > 180.0):
        return "turn right", 360.0-diffAngle


#-----------------------------------------------------------------------------
# update current coordinate based on the starting coordinate, starting
# direction and number of steps taken in that direction
#-----------------------------------------------------------------------------
def updateCurrentLocation(prevX, prevY, prevHeading, numStep):
    # 1 step: 30cm
    r = numStep * stepLength
    
    if (prevHeading <= 90): #1st quadrant
        currentX = prevX + r * math.sin(math.radians(prevHeading))
        currentY = prevY + r * math.cos(math.radians(prevHeading))
        
    elif (90.0 < prevHeading and prevHeading <= 180.0): #4th quadrant
        currentX = prevX + r * math.sin(math.radians(180.0 - prevHeading))
        currentY = prevY - r * math.cos(math.radians(180.0 - prevHeading))
        
    elif (180.0 < prevHeading and prevHeading <= 270.0): #3rd quadrant
        currentX = prevX - r * math.sin(math.radians(prevHeading - 180.0))
        currentY = prevY - r * math.cos(math.radians(prevHeading - 180.0))
        
    elif (270.0 < prevHeading and prevHeading < 360.0): #2nd quadrant
        currentX = prevX - r * math.sin(math.radians(360.0 - prevHeading))
        currentY = prevY + r * math.cos(math.radians(360.0 - prevHeading))
        
    return currentX, currentY


#-----------------------------------------------------------------------------
# Returns shortest path as a list of nodes given a graph, start node, end node
# using Dijkstra's algorithm. Reference: 
# http://pyalgoviz.appspot.com/show?edit=False&name=Graphs%20-%20Dijkstra%20Shortest%20Path
#-----------------------------------------------------------------------------
def shortestPath(graph, start, end):
    queue = [(0, start, [])]
    seen = set()
    while True:
        (cost, v, path) = heapq.heappop(queue)
        if v not in seen:
            path = path + [v]
            seen.add(v)
            if v == end:
                return cost, path
            for (next, c) in graph[v].iteritems():
                heapq.heappush(queue, (cost + c, next, path))

#-----------------------------------------------------------------------------
main()