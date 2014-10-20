##!/usr/bin/env python
#from scapy.all import *
 
#def PacketHandler(pkt) :
 
#    if pkt.haslayer(Dot11) :
#        if pkt.type == 0 and pkt.subtype == 8 :
#            print "AP MAC: %s with SSID: %s " %(pkt.addr2, pkt.info)
    
    
#sniff(iface="mon0", prn = PacketHandler)

import math

# reference:
# http://stackoverflow.com/questions/16176656/trilateration-and-locating-the-point-x-y-z
def getTrilateration(position1, position2, position3):
    
    x1 = position1.x
    y1 = position1.y
    x2 = position2.x
    y2 = position2.y
    x3 = position3.x
    y3 = position3.y
    r1 = position1.r
    r2 = position2.r
    r3 = position3.r
 
    #translate position 1 to origin
    x1 = x1 - x1
    y1 = y1 - y1
    x2 = x2 - x1
    y2 = y2 - y1
    x3 = x3 - x1
    y3 = y3 - y1
    
    #rotate position2 translated vector to x-axis if it's not so
    magnitude2 = math.sqrt(math.pow(x2, 2.) + math.pow(y2, 2.))
    theta =  math.acos(x2 / magnitude2)
    
    if (y2 > 0):    #if position2 is above x-axis, rotate clockwise, else rotate anti clockwise
        theta = theta * -1.0
    
    x2 = x2 * math.cos(theta) - y2 * math.sin(theta)
    y2 = x2 * math.sin(theta) + y2 * math.cos(theta)
    
    x3 = x3 * math.cos(theta) - y3 * math.sin(theta)
    y3 = x3 * math.sin(theta) + y3 * math.cos(theta)

    #solve for position4 (location)
    x4 = (math.pow(r1, 2.) - math.pow(r2, 2.) + math.pow(x2, 2.)) / (x2 * 2.)
    y4 = (math.pow(r1, 2.) - math.pow(r3, 2.) + math.pow(x3, 2.) + math.pow(y3, 2.)) / (2.0 * y3 - (x3/y3) * x4)
    
    #rotate the result back
    x4 = x4 * math.cos(-1 * theta) - y4 * math.sin(-1 * theta)
    y4 = x4 * math.sin(-1 * theta) + y4 * math.cos(-1 * theta)
    
    #translate the result back
    x4 = x4 + position1.x
    y4 = y4 + position1.y

    return x4, y4


class Position(object):
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.r = radius #radius
        

