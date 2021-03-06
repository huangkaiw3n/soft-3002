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
# String to hold data from uart
inStr = "" 
#-----------------------------------------------------------------------------
# Sample command to play Intro.txt audio
# os.system('espeak -v+f3 -s150 -f /home/pi/Vision/Intro.txt --stdout | aplay')
# all audio will be put in .txt files to resolve a lag issue with audio playback
# For playback without .txt file:
# os.system('espeak -v+f3 -s150 "your command here" --stdout | aplay')
#-----------------------------------------------------------------------------
# UART peripherals
Wire.wiringPiSetupGpio()
GPIO.setmode(GPIO.BCM)
GPIO.setup(25, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(24, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
serialPort = Wire.serialOpen("/dev/ttyAMA0", 4800)

inStr = ""
myList = []
elapsed = 0
elapsed2 = 0
bufferIndex = 0
dataSend = ""

#-----------------------------------------------------------------------------
# initialise building name and level
# get JSON response from the url and parse to data
#-----------------------------------------------------------------------------
def initialise():
    os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/StartingPrompt.txt --stdout | aplay')

    control = ""
    sendKeyInt()
    
    while control != "1#" and control != "2#" and control != "3#" and control != "4#" :
        if control != "":
            say("Please enter 1, 2, 3 or 4 only.")
            sendKeyInt()
        UART_Buffer()
        control = getKeyData()

    buildingName = ""
    level = ""
    while control == "1#":
        os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/BuildingName.txt --stdout | aplay')
        sendKeyInt()
        while buildingName == "" :
            UART_Buffer()
            buildingName = getKeyData()[:-1]
            
        os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/BuildingLevel.txt --stdout | aplay')
        sendKeyInt()
        
        while (True):
            while level == "" :
                UART_Buffer()
                level = getKeyData()[:-1]
                
            try:
                params = dict(Building = buildingName, Level = int(level))
                break
            except ValueError:
                os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/InvalidLevel.txt --stdout | aplay')
                level = ""
                sendKeyInt()
        
        mapID = [buildingName, int(level)]
        if (internet_on() == True):
            response = requests.get(url=url, params = params)
        else:
            time.sleep(5)
            if (internet_on() == False):
                os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/InternetOff.txt --stdout | aplay')
                control = "2#"
                break
            else:
                response = requests.get(url=url, params = params)
        data = response.json()
        if data["info"] is not None:
            #map name loaded
            break
        os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/WrongMapInfo.txt --stdout | aplay')
        buildingName = ""
        level = ""
        
    if control == "2#":
        json_data=open('/home/pi/soft-3002/Software3002/Maps/com1_2.txt')
        data = json.load(json_data)
        json_data.close()

    elif control == "3#":
        json_data=open('/home/pi/soft-3002/Software3002/Maps/com2_2.txt')
        data = json.load(json_data)
        json_data.close()

    elif control == "4#":
        json_data=open('/home/pi/soft-3002/Software3002/Maps/com2_3.txt')
        data = json.load(json_data)
        json_data.close()
  
    #FloorPlanDatabase = FloorPlanList()  #initialise database
    #myFloorPlan = FloorPlan(mapID, data)    
    #FloorPlanDatabase.addFloorPlan(myFloorPlan) #add floor plan to database
    
    return data

#-----------------------------------------------------------------------------
# check whether the destination is reached.
# return True if current location is within 0.5 meter radius of destination
#-----------------------------------------------------------------------------
def isReached(targetNode):
    
    d = getDistance(targetNode)
        
    if (d < offsetDistance):
        return True
    else:
        return False
     
def getDistance(targetNode):
    yDistance = math.fabs((targetNode.y * 1.0 - currentY * 1.0) * 1.0)
    xDistance = math.fabs((targetNode.x * 1.0 - currentX * 1.0) * 1.0)
    
    d = math.sqrt(yDistance ** 2 + xDistance ** 2)
    print ("distance: " + str(d) + "\n")

    return d

#-----------------------------------------------------------------------------
# compute direction based on current coordinate and heading,
# since currentHeading is with respect to the map's North, it will be offset
# to become with respect to the absolute north.
#-----------------------------------------------------------------------------    
def computeDirection(targetNode, north):
    currentHeadingWithOffset = currentHeading + north
    if (currentHeadingWithOffset >= 360.0):
        currentHeadingWithOffset = currentHeadingWithOffset - 360.0

    yDistance = (targetNode.y * 1.0 - currentY * 1.0) * 1.0
    xDistance = (targetNode.x * 1.0 - currentX * 1.0) * 1.0
    
    if (yDistance == 0):
        if (targetNode.x * 1.0 - currentX * 1.0) > 0.0:
            targetHeading = 90.0
        else:
            targetHeading = 270.0
    elif (xDistance == 0):
        if (targetNode.y * 1.0 - currentY * 1.0) > 0.0:
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
        return finalHeading
    elif (-180.0 < diffAngle and diffAngle < -offSetAngle):
        finalHeading = currentHeading + (math.fabs(diffAngle))
        if finalHeading >= 360:
            finalHeading = finalHeading - 360
        return finalHeading
    elif (-offSetAngle < diffAngle and diffAngle < offSetAngle):
        return currentHeading
    elif (offSetAngle < diffAngle and diffAngle < 180.0):
        finalHeading = currentHeading - diffAngle
        if finalHeading < 0:
            finalHeading = finalHeading + 360
        return finalHeading
    elif (diffAngle > 180.0):
        finalHeading = currentHeading + (360.0-diffAngle)
        if finalHeading >= 360:
            finalHeading = finalHeading - 360
        return finalHeading

def directUser(angleToFace):
    try:
        diff = currentHeading - angleToFace
    except TypeError:
        return ""

    if(math.fabs(diff) <= offSetAngle):
        return "straight"
    if(offSetAngle < diff and diff <= 180):
        return "turn left"
    if(180 < diff and diff <= 360):
        return "turn right"
    if(-180 <= diff and diff < (offSetAngle * -1)):
        return "turn right"
    if(-360 <= diff and diff < -180):
        return "turn left"


#-----------------------------------------------------------------------------
# update current coordinate based on the starting coordinate, starting
# direction and number of steps taken in that direction
#-----------------------------------------------------------------------------
#def updateCurrentLocation(prevX, prevY, prevHeading, numStep):
#    # 1 step: 30cm
#    r = numStep * stepLength
    
#    if (prevHeading <= 90): #1st quadrant
#        currentX = prevX + r * math.sin(math.radians(prevHeading))
#        currentY = prevY + r * math.cos(math.radians(prevHeading))
        
#    elif (90.0 < prevHeading and prevHeading <= 180.0): #4th quadrant
#        currentX = prevX + r * math.sin(math.radians(180.0 - prevHeading))
#        currentY = prevY - r * math.cos(math.radians(180.0 - prevHeading))
        
#    elif (180.0 < prevHeading and prevHeading <= 270.0): #3rd quadrant
#        currentX = prevX - r * math.sin(math.radians(prevHeading - 180.0))
#        currentY = prevY - r * math.cos(math.radians(prevHeading - 180.0))
        
#    elif (270.0 < prevHeading and prevHeading < 360.0): #2nd quadrant
#        currentX = prevX - r * math.sin(math.radians(360.0 - prevHeading))
#        currentY = prevY + r * math.cos(math.radians(360.0 - prevHeading))
        
#    return currentX, currentY


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
                return path  #remove "cost"
            for (next, c) in graph[v].iteritems():
                heapq.heappush(queue, (cost + c, next, path))

# UART functions below-------------------------------------------------------

def readUART():
    ch = ""
    tempStr = ""
    global inStr
    elapsed = Wire.millis()
    
    while(ch != 123):
        if(Wire.serialDataAvail(serialPort) > 0):
            ch = Wire.serialGetchar(serialPort)
            if(ch < 123 and ch > -1):
                print ch
                tempStr += chr(ch)
        if(Wire.millis() - elapsed > 3000):
            elapsed = Wire.millis()
            ch = 123
    inStr = tempStr
    return

def putBuffer():
    global bufferIndex
    if(inStr != ""):
        expression = (30 - int(bufferIndex)) + 1
        if(len(inStr) < expression):
            myList[bufferIndex:len(inStr)] = inStr
            bufferIndex = bufferIndex + len(inStr)
    return

def sendKeyInt():
    GPIO.output(25, True)
    Wire.delayMicroseconds(100)
    GPIO.output(25, False)
    return

def sendSensorInt():
    GPIO.output(24, True)
    Wire.delayMicroseconds(100)
    GPIO.output(24, False)
    return

def flushBuffer():
    global bufferIndex
    Wire.serialFlush(serialPort)
    myList[0:] = []
    bufferIndex = 0
    return

def getKeyData():
    dataStr = ""
    if (myList != []):
        for x in xrange(0,len(myList)):
            dataStr += myList[x]
            if(myList[x] == "#"):
                flushBuffer()
                return dataStr
        return ""
    else:
        return ""

def getLocData():
    global bufferIndex
    global currentX
    global currentY
    global currentHeading
    iterate = len(myList) / 5

    try:
        if(myList != []):
            for x in xrange(0, iterate):
                index = x*5
                idStr = ""
                sign = 0
                dataStr1 = ""
                dataStr2 = ""
                dataStr3 = ""
                realData = 0
                idStr = ord(myList[index])
                sign = ord(myList[index+1])
                dataStr1 = myList[index+2]
                dataStr2 = myList[index+3]
                dataStr3 = myList[index+4]
                realData += (ord(dataStr1))*10000
                realData += (ord(dataStr2))*100
                realData += (ord(dataStr3))
                if((realData is not None)):
                    if(sign == 1):
                        realData *= -1
                    elif(idStr == 18):
                        currentX = realData
                    elif(idStr == 28):
                        currentY = realData
                    elif(idStr == 38):
                        currentHeading = realData
    except TypeError:
        flushBuffer()
        print("Type Error, Probably list is non-iterable")
        return
    
    except ValueError:
        flushBuffer()
        print("Char ascii value not valid")
        return
    else:
        flushBuffer()
        print currentX
        print currentY
        print currentHeading
    return


def parseInfo(iden, data):
    global dataSend
    dataSend = ""
    dataSend = str(unichr(iden))
    Wire.serialPuts(serialPort, dataSend)
    if(int(data/10000) == 0):
        Wire.serialPutchar(serialPort, 00)
    else:
        dataSend = str(unichr(int(data/10000)))
        Wire.serialPuts(serialPort, dataSend)
    if(int((data/100)%100) == 0):
        Wire.serialPutchar(serialPort, 00)
    else:
        dataSend = str(unichr(int((data/100)%100)))
        Wire.serialPuts(serialPort, dataSend)
    if(int(data%100) == 0):
        Wire.serialPutchar(serialPort, 00)
    else:
        dataSend = str(unichr(int(data%100)))
        Wire.serialPuts(serialPort, dataSend)
    Wire.serialPutchar(serialPort, 123)
    return

def UART_Buffer():
    while(Wire.serialDataAvail(serialPort) > 0):
        readUART()
        putBuffer()
    return
     
#GPIO.add_event_detect(25, GPIO.RISING, callback=readUART)
def handShake():
    global elapsed, elapsed2
    Wire.serialFlush(serialPort)
    Wire.serialPuts(serialPort, "Pi Booty")
    Wire.serialPutchar(serialPort, 123)
    elapsed = Wire.millis()
    elapsed2 = Wire.millis()
    while(inStr != "Ack"):
    
        time.sleep(0.5)
        readUART()
        time.sleep(0.5)
        if(Wire.millis() - elapsed2 > 5000):
            elapsed2 = Wire.millis()
            Wire.serialPuts(serialPort, "Pi Booty")
            Wire.serialPutchar(serialPort, 123)
    return

# Espeak----------------------------------------------------------------------
def say(something):
    os.system('espeak -v+f3 -s150 "{0}" --stdout |aplay'.format(something))

def infoReport(targetNode):

    d = getDistance(targetNode)
    steps = int(d/stepLength)
    say(str(steps) + "steps.")
    # + "steps to " + str(targetNode.id) + " " + targetNode.name

    return steps

#-----------------------------------------------------------------------------
# main
#-----------------------------------------------------------------------------
def main():
    global currentX
    global currentY
    global currentHeading

    os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/AttemptHandShake.txt --stdout | aplay')
    handShake()
    os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/HandShakeCompleted.txt --stdout | aplay')
    os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/Intro.txt --stdout | aplay')
    
    data = initialise()
    locationNodeList = LocationNodeList(data["map"], data["info"])
    wifiNodeList = WifiNodeList(data["wifi"])
    aList = AdjList(locationNodeList)        #constructs adjlist from node list
 
    while (True):   #edited to return here when a navigation has completed
        currentInstruction = ""
        os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/Begin.txt --stdout | aplay')
        flushBuffer()
        sendKeyInt()
        while currentInstruction != "1#" and currentInstruction != "2#": #yet to complete change yet. #2 will be change map.
            if currentInstruction != "":
                say("Please press 1 or 2 only.")
                sendKeyInt()     
            UART_Buffer()
            currentInstruction = getKeyData()

        if currentInstruction == "2#":
            data = initialise()
            locationNodeList = LocationNodeList(data["map"], data["info"])
            wifiNodeList = WifiNodeList(data["wifi"])
            aList = AdjList(locationNodeList)        #constructs adjlist from node list

        else:
            # Gets current node id and dest. node id from user, then asks user to confirm if it's correct--------
            nodeConfirm = ""
            while (nodeConfirm != "1#"):
                startNodeId = -1
                destinationNodeId = -1
                temp = ""
                os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/CurrentNode.txt --stdout | aplay')
                sendKeyInt()
                while (True):
                    if temp != "":
                        try:   
                            startNodeId = int(temp[:-1])
                            if (startNodeId in locationNodeList.list):
                                break
                            else:
                                os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/WrongNodeId.txt --stdout | aplay')
                                sendKeyInt()
                        except ValueError:
                            os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/InvalidNodeID.txt --stdout | aplay')
                            sendKeyInt()
                    UART_Buffer()
                    temp = getKeyData()

                temp = ""
                os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/DesNode.txt --stdout | aplay')
                sendKeyInt()
                while (True):
                    if temp != "":
                        try:
                            destinationNodeId = int(temp[:-1])
                            if (destinationNodeId == startNodeId):
                                say("Destination Node cannot be same as Start Node. Please re-enter.")
                                sendKeyInt()
                            elif (destinationNodeId in locationNodeList.list):
                                break
                            else:
                                os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/WrongNodeId.txt --stdout | aplay')
                                sendKeyInt()
                        except ValueError:
                            os.system('espeak -v+f3 -s150 -f /home/pi/soft-3002/Software3002/Audio/InvalidNodeID.txt --stdout | aplay')
                            sendKeyInt()
                    UART_Buffer()
                    temp = getKeyData()

                say("Node " + str(startNodeId) + " to " + "Node " + str(destinationNodeId))
                say("To confirm, press 1, otherwise, press 2 to re-enter.")
                nodeConfirm = ""
                sendKeyInt()
                while nodeConfirm != "1#" and nodeConfirm != "2#" :
                    if nodeConfirm != "":
                        say("Please enter 1 or 2 only.")
                        sendKeyInt()
                    UART_Buffer()
                    nodeConfirm = getKeyData()
# ----------------------------------------------------------------------------------------------------
# Assigns the startNode and destinationNode using the id given by user, maps the shortest path--------
            startNode = locationNodeList.getNodeById(startNodeId)
            destinationNode = locationNodeList.getNodeById(destinationNodeId)
        
            path = shortestPath(aList.getGraph(), startNode.id, destinationNode.id)
            currentX = startNode.x
            currentY = startNode.y
# -----------------------------------------------------------------------------------------------------
# Wait for user to prompt begin. (after keying the nodes and being spinned)----------------------------
            beginNav = ""
            say("Press one to start navigation")
            sendKeyInt()
            while beginNav != "1#": #for user to begin after keying in start and end.
                if beginNav != "":
                    say("Press one to start navigation")
                    sendKeyInt()
                UART_Buffer()
                beginNav = getKeyData()
# ------------------------------------------------------------------------------------------------------
# Send initial values of x, y and map north to arduino--------------------------------------------------
            sendSensorInt()   #indicate to ard to read sensor, then send ard currentXY
            parseInfo(18, currentX)
            parseInfo(28, currentY)
            i = 1
            startTime = time.time()

            leftFlag = 0
            rightFlag = 0
            time.sleep(1) #allow time for arduino to get average from heading.

# -------------------------------------------------------------------------------------------------------
# Navigation Loop----------------------------------------------------------------------------------------
            while (isReached(destinationNode) == False):

                currentNode = locationNodeList.getNodeById(path[i-1])
                targetNode = locationNodeList.getNodeById(path[i])
                say("From " + currentNode.name + ", " + "to " + targetNode.name)
                while(True):
                    try:
                        mapDegree  = computeDirection(targetNode, locationNodeList.north)
                        break;
                    except TypeError:
                        print("currentHeading:", currentHeading, "\n currentX:", currentX, "\n currentY:", currentY, "\n targetnodeXcord:", targetNode.x, "\n targetnodeYcord:", targetNode.y, "\n MapNorth:", locationNodeList.north) 
                        print("end of data\n")
                        break;
                mapDegree = (mapDegree + locationNodeList.north) % 360
                parseInfo(48, mapDegree)

                angleToFace = int(currentNode.neighbourAngle[str(path[i])])
                print ("AngletoFace:" + str(angleToFace))

                time.sleep(1)
                direction = ""
                while(direction != "straight"):
                    UART_Buffer()
                    getLocData() #update currentXYH
                    direction = directUser(angleToFace)
                    print ("CurrentHeading: " + str(currentHeading) + "\n")

                    if direction == "turn left" and leftFlag == 0:
                        leftFlag = 1
                        say("Left")
                    elif direction == "turn right" and rightFlag == 0:
                        rightFlag = 1
                        say("Right")
                    elif direction == "straight":
                        leftFlag = 0
                        rightFlag = 0
                        print("TargetNode: ")
                        print(targetNode.id)
                        print("\n CurrentHeading: ")
                        print(currentHeading)
                        print("\n CurrentX: ")
                        print(currentX)
                        print("\n CurrentY: ")
                        print(currentY)
                        say("Straight")
                    elif leftFlag == 1 and rightFlag == 1:
                        leftFlag = 0
                        rightFlag = 0

                prevStep = 1000000
                while (isReached(targetNode) == False):
                    UART_Buffer()
                    getLocData() #update currentXYH

                    if(time.time() - startTime > reportInterval):
                        currentStep = infoReport(targetNode)
                        startTime = time.time()
                    #print("PrevStep: " + str(prevStep) + "\n")
                    #print("CurrentStep: " + str(currentStep) + "\n")
                    #time.sleep(0.5)
                    if(prevStep < currentStep):
                        break
                    else:
                        prevStep = currentStep

                    UART_Buffer()
                    getLocData() #update currentXYH
           
                #output to audio here: 
                say("Reached Node " + str(path[i]) + " " + targetNode.name)

                if targetNode != destinationNode : #prevent overflow
                    i = i + 1 

            say("Reached destination " + destinationNode.name)
            sendSensorInt() #stop the sensors on arduino
# ----------------------------------------------------------------------------------------------------
main()