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
# os.system('espeak -v+f3 -s100 -f /home/pi/Vision/Intro.txt --stdout | aplay')
# all audio will be put in .txt files to resolve a lag issue with audio playback
# For playback without .txt file:
# os.system('espeak -v+f3 -s100 "your command here" --stdout | aplay')
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
    global inStr
    os.system('espeak -v+f3 -s100 -f /home/pi/soft-3002/Software3002/Audio/Intro.txt --stdout | aplay')
    os.system('espeak -v+f3 -s100 -f /home/pi/soft-3002/Software3002/Audio/StartingPrompt.txt --stdout | aplay')

    control = ""
    sendKeyInt()
    while control != "1#" and control != "2#" :
        if control != "":
            sendKeyInt()
        UART_Buffer()
        control = getKeyData()

    buildingName = ""
    level = ""
    while control == "1#":
        sendKeyInt()
        os.system('espeak -v+f3 -s100 -f /home/pi/soft-3002/Software3002/Audio/BuildingName.txt --stdout | aplay')
        while buildingName == "" :
            UART_Buffer()
            buildingName = getKeyData()[:-1]
        sendKeyInt()
        os.system('espeak -v+f3 -s100 -f /home/pi/soft-3002/Software3002/Audio/BuildingLevel.txt --stdout | aplay')
        while level == "" :
            UART_Buffer()
            level = getKeyData()[:-1]
        mapID = [buildingName, int(level)]
        params = dict(Building = buildingName, Level = int(level))
        if (internet_on() == True):
            response = requests.get(url=url, params = params)
        else:
            time.sleep(5)
            if (internet_on() == False):
                os.system('espeak -v+f3 -s100 -f /home/pi/soft-3002/Software3002/Audio/InternetOff.txt --stdout | aplay')
                control = "2#"
                break
            else:
                response = requests.get(url=url, params = params)
        data = response.json()
        if data["info"] is not None:
            break
        os.system('espeak -v+f3 -s100 -f /home/pi/soft-3002/Software3002/Audio/WrongMapInfo.txt --stdout | aplay')
        
    if control == "2#":
        json_data=open('com.txt')
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
        expression = (20 - int(bufferIndex)) + 1
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

def getKeyData():
    dataStr = ""
    if (myList != []):
        for x in xrange(0,len(myList)):
            dataStr += myList[x]
            if(myList[x] == "#"):
                myList[0:] = []
                return dataStr
        return ""
    else:
        return ""

def getLocData():
    global bufferIndex
    global currentX
    global currentY
    global currentHeading
    iterate = len(myList) / 4
    if (myList != []):
        for x in xrange(0,iterate):
            index = x*4
            idStr = ""
            dataStr1 = ""
            dataStr2 = ""
            dataStr3 = ""
            realData = 0
            idStr = ord(myList[index])
            dataStr1 = myList[index+1]
            dataStr2 = myList[index+2]
            dataStr3 = myList[index+3]
            realData += (ord(dataStr1))*10000
            realData += (ord(dataStr2))*100
            realData += (ord(dataStr3))
            if(idStr == 18):
                currentX = realData
            elif(idStr == 28):
                currentY = realData
            elif(idStr == 38):
                currentHeading = realData
        myList[0:] = []
        bufferIndex = 0
        print currentX
        print currentY
        print currentHeading
    return

def parseInfo(iden, data):
    global dataSend
    dataSend = ""
    #dataSend = dataSend + str(iden)
    #dataSend = dataSend + str(data/10000)
    #dataSend = dataSend + str(unichr((data/100)%100))
    #dataSend = dataSend + str(unichr(data%100))
    dataSend = str(unichr(iden))
    Wire.serialPuts(serialPort, dataSend)
    if(data/10000 == 0):
        Wire.serialPutchar(serialPort, 00)
    else:
        dataSend = str(unichr(int(data/10000)))
        Wire.serialPuts(serialPort, dataSend)
    if((data/100)%100 == 0):
        Wire.serialPutchar(serialPort, 00)
    else:
        dataSend = str(unichr(int((data/100)%100)))
        Wire.serialPuts(serialPort, dataSend)
    if(data%100 == 0):
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
    os.system('espeak -v+f3 -s100 "{0}" --stdout |aplay'.format(something))
#-----------------------------------------------------------------------------
# main
#-----------------------------------------------------------------------------
def main():
    os.system('espeak -v+f3 -s100 -f /home/pi/soft-3002/Software3002/Audio/AttemptHandShake.txt --stdout | aplay')
    handShake()
    os.system('espeak -v+f3 -s100 -f /home/pi/soft-3002/Software3002/Audio/HandShakeCompleted.txt --stdout | aplay')
    data = initialise()
    #pprint(data) #to be removed
    global currentX
    global currentY
    global currentHeading

    locationNodeList = LocationNodeList(data["map"], data["info"])
    wifiNodeList = WifiNodeList(data["wifi"])
    
    aList = AdjList(locationNodeList)        #constructs adjlist from node list

    currentInstruction = ""
    os.system('espeak -v+f3 -s100 -f /home/pi/soft-3002/Software3002/Audio/Begin.txt --stdout | aplay')
    sendKeyInt()
    while currentInstruction != "1#":
        if currentInstruction != "":
            sendKeyInt()     
        UART_Buffer()
        currentInstruction = getKeyData()
    
    while (currentInstruction == "1#"):   #edited to return here when a navigation has completed
        startNodeId = -1
        destinationNodeId = -1
        temp = ""
        os.system('espeak -v+f3 -s100 -f /home/pi/soft-3002/Software3002/Audio/CurrentNode.txt --stdout | aplay')
        sendKeyInt()
        while (True):
            if temp != "":
                startNodeId = int(temp[:-1])
                if (startNodeId in locationNodeList.list):
                    break
                else:
                    os.system('espeak -v+f3 -s100 -f /home/pi/soft-3002/Software3002/Audio/WrongNodeId.txt --stdout | aplay')
                    sendKeyInt()
            UART_Buffer()
            temp = getKeyData()

        temp = ""
        os.system('espeak -v+f3 -s100 -f /home/pi/soft-3002/Software3002/Audio/DesNode.txt --stdout | aplay')
        sendKeyInt()
        while (True):
            if temp != "":
                destinationNodeId = int(temp[:-1])
                if (destinationNodeId in locationNodeList.list):
                    break
                else:
                    os.system('espeak -v+f3 -s100 -f /home/pi/soft-3002/Software3002/Audio/WrongNodeId.txt --stdout | aplay')
                    sendKeyInt()
            UART_Buffer()
            temp = getKeyData()

        startNode = locationNodeList.getNodeById(startNodeId)
        destinationNode = locationNodeList.getNodeById(destinationNodeId)
        
        cost, path = shortestPath(aList.getGraph(), startNode.id, destinationNode.id)
        currentX = startNode.x
        currentY = startNode.y

        sendSensorInt()   #indicate to ard to read sensor, then send ard currentXY
        parseInfo(18, currentX)
        parseInfo(28, currentY)
        parseInfo(48, locationNodeList.north)
        i = 1

        while (isReached(currentX, currentY, destinationNode.x, destinationNode.y) == False):
            
            targetNode = locationNodeList.getNodeById(path[i])
            
            while (isReached(currentX, currentY, targetNode.x, targetNode.y) == False):
  
                time.sleep(1) #assume 1 step 1 second
                UART_Buffer()
                getLocData() #update currentXYH
               
                direction, degree  = computeDirection(currentHeading, currentX, currentY, targetNode.x, targetNode.y, locationNodeList.north)
                print (direction, degree)
                if direction == "turn left":
                    parseInfo(19, degree)
                elif direction == "turn right":
                    parseInfo(29, degree)
                else:
                    os.system('espeak -v+f3 -s100 -f /home/pi/soft-3002/Software3002/Audio/GoStraight.txt --stdout | aplay')

                #just for testing. to be updated through uart by arduino
                #currentX = int(input("Input currentX\n"))
                #currentY = int(input("Input currentY\n"))
                #currentHeading = int(input("Input currentHeading\n"))
                #currentX, currentY = updateCurrentLocation(currentX, currentY, currentHeading, 1)
            
            #output to audio here: 
            temp = "Reached Node " + str(path[i])
            say(temp)

            if targetNode != destinationNode : #prevent overflow
                i = i + 1 
    #else:
     #   print ("test") 



main()