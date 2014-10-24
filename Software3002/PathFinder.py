# main module of the system
from Settings import *
from Uart import *

#readUART() - reads the receive buffer and returns a string
#putBuffer() puts the string returned by readUART into the buffer
#getLocData() -updates currentXYHeading
#readKeypad() -returns a string with # at the end. eg. "12#"
#sendKeyInt() -indicates to ard to read keypad. Needs to be called everytime when keypad needs to be read
#sendSensorInt() -indicates to ard the start of navigation. to be called before sending ard x y coordinates.
#

# rPi GPIO interrupt will activate UART read on rPi's side
# flag to tell UART handler whether we are taking in currentXYheading during
# navigation or we are taking in keyad values when outside navigation
navigation = 0

#-----------------------------------------------------------------------------
# Sample command to play Intro.txt audio
# os.system(espeak -v+f3 -s100 -f /home/pi/Vision/Intro.txt --stdout | aplay)
# all audio will be put in .txt files to resolve a lag issue with audio playback
# For playback without .txt file:
# os.system(espeak -v+f3 -s100 "your command here" --stdout | aplay)
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# initialise building name and level
# get JSON response from the url and parse to data
#-----------------------------------------------------------------------------
def initialise():
    global inStr
    os.system(espeak -v+f3 -s100 -f /home/pi/Vision/Intro.txt --stdout | aplay)
    os.system(espeak -v+f3 -s100 -f /home/pi/Vision/StartingPrompt.txt --stdout | aplay)

    control = ""
    sendKeyInt()
    while control != "1#" and control != "2#" :
        if control != "":
            sendKeyInt()
        inStr = readUART()
        putBuffer()
        control = readKeypad()

    buildingName = ""
    level = ""
    while control == "1#":
        sendKeyInt()
        while buildingName == "" :
            inStr = readUART()
            putBuffer() 
            buildingName = readKeypad()[:-1]
        sendKeyInt()
        while level == "" :
            inStr = readUART()
            putBuffer()
            level = readKeypad()[:-1]
        mapID = [buildingName, int(level)]
        params = dict(Building = buildingName, Level = int(level))
        if (internet_on() == True):
            response = requests.get(url=url, params = params)
        else:
            time.sleep(5)
            if (internet_on() == False):
                os.system(espeak -v+f3 -s100 -f /home/pi/Vision/InternetOff.txt --stdout | aplay)
                control = "2#"
                break
            else:
                response = requests.get(url=url, params = params)
        data = response.json()
        if data["info"] is not None:
            break
        os.system(espeak -v+f3 -s100 -f /home/pi/Vision/WrongMapInfo.txt --stdout | aplay)
        
    if control == "2#":
        json_data=open('com.txt')
        data = json.load(json_data)
        json_data.close()
  
    #FloorPlanDatabase = FloorPlanList()  #initialise database
    #myFloorPlan = FloorPlan(mapID, data)    
    #FloorPlanDatabase.addFloorPlan(myFloorPlan) #add floor plan to database
    
    return data


#-----------------------------------------------------------------------------
# main
#-----------------------------------------------------------------------------
def main():
    data = initialise()
    pprint(data) #to be removed
    global currentX
    global currentY
    global currentHeading

    locationNodeList = LocationNodeList(data["map"], data["info"])
    wifiNodeList = WifiNodeList(data["wifi"])
    
    aList = AdjList(locationNodeList)        #constructs adjlist from node list

    currentInstruction = ""
    os.system(espeak -v+f3 -s100 -f /home/pi/Vision/Begin.txt --stdout | aplay)
    sendKeyInt()
    while currentInstruction != "1#":
        if currentInstruction != "":
            sendKeyInt()     
        inStr = readUART()
        putBuffer() 
        currentInstruction = readKeypad()
    
    if (currentInstruction == "1#"):
        startNodeId = -1
        destinationNodeId = -1
        temp = ""
        os.system(espeak -v+f3 -s100 -f /home/pi/Vision/CurrentNode.txt --stdout | aplay)
        sendKeyInt()
        while (True):
            if temp != "":
                startNodeId = int(temp[:-1])
                if (startNodeId in locationNodeList.list):
                    break
                else:
                    os.system(espeak -v+f3 -s100 -f /home/pi/Vision/WrongNodeId.txt --stdout | aplay)
                    sendKeyInt()
            inStr = readUART()
            putBuffer() 
            temp = readKeypad()

        temp = ""
        while (True):
            if temp != "":
                destinationNodeId = int(temp[:-1])
                if (destinationNodeId in locationNodeList.list):
                    break
                else:
                    os.system(espeak -v+f3 -s100 -f /home/pi/Vision/WrongNodeId.txt --stdout | aplay)
                    sendKeyInt()
            inStr = readUART()
            putBuffer() 
            temp = readKeypad()

        startNode = locationNodeList.getNodeById(startNodeId)
        destinationNode = locationNodeList.getNodeById(destinationNodeId)
        
        cost, path = shortestPath(aList.getGraph(), startNode.id, destinationNode.id)
        currentX = startNode.x
        currentY = startNode.y

        sendSensorInt()   #indicate to ard to read sensor, then send ard currentXY
        #sendXY
        i = 1

        while (isReached(currentX, currentY, destinationNode.x, destinationNode.y) == False):
            
            targetNode = nodeList.getNodeById(path[i])
            
            while (isReached(currentX, currentY, targetNode.x, targetNode.y) == False):
  
                time.sleep(1) #assume 1 step 1 second
                getLocData() #update currentXYH
               
                direction, degree  = computeDirection(currentHeading, currentX, currentY, targetNode.x, targetNode.y, nodeList.north)
                print (direction, degree)

                #just for testing. to be updated through uart by arduino
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
        finalHeading = currentHeading - (360.0-math.fabs(diffAngle))
        if finalHeading < 0:
            finalHeading = finalHeading + 360
        return "turn left", finalHeading
    elif (-180.0 < diffAngle and diffAngle < -offSetAngle):
        finalHeading = currentHeading + (math.fabs(diffAngle))
        if finalHeading >= 360:
            finalHeading = finalHeading - 360
        return "turn right", finalHeading
    elif (-offSetAngle < diffAngle and diffAngle < offSetAngle):
        return "straight", currentHeading
    elif (offSetAngle < diffAngle and diffAngle < 180.0):
        finalHeading = currentHeading - diffAngle
        if finalHeading < 0:
            finalHeading = finalHeading + 360
        return "turn left", finalHeading
    elif (diffAngle > 180.0):
        finalHeading = currentHeading + (360.0-diffAngle)
        if finalHeading >= 360:
            finalHeading = finalHeading - 360
        return "turn right", finalHeading

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