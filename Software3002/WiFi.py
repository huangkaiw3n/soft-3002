##!/usr/bin/env python
#from scapy.all import *
 
#def PacketHandler(pkt) :
 
#    if pkt.haslayer(Dot11) :
#        if pkt.type == 0 and pkt.subtype == 8 :
#            print "AP MAC: %s with SSID: %s " %(pkt.addr2, pkt.info)
    
    
#sniff(iface="mon0", prn = PacketHandler)

class Test:
    a = 2
    b = 3

    def __init__(self, a):
        print self.a
        self.a = a
        print self.a
        self.__x = 123;
        self.__y = 123;
        b = 'meow'

    def gety(self):
        return self.__y

#main()

test = Test(3)

print test.__x


