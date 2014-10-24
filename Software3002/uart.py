from Settings import currentX, currentY, currentHeading

# This module implements Uart Communication
#getLocData() -updates currentXYHeading
#readKeypad() -returns a string with # at the end. eg. "12#"
#sendKeyInt() -indicates to ard to read keypad. Needs to be called everytime when keypad needs to be read
#sendSensorInt() -indicates to ard the start of navigation. to be called before sending ard x y coordinates.

def getLocData():
    #global call to enable editing of global variable
    global currentX
    global currentY
    global currentHeading

    return
    
def readKeypad():
    
    return "stub string"

def sendKeyInt():

    return 

def sendSensorInt():
    
    return

def readUART():

    return "stub string"

def putBuffer():

    return