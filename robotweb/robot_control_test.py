#! /usr/bin/env python

import time
import mini_driver

miniDriver = mini_driver.MiniDriver()
connected = miniDriver.connect()
print "connected =", connected

if connected:
    
    miniDriver.setOutputs( 50, 50, 30, 150 )
    time.sleep( 1.0 )
    
    miniDriver.setOutputs( -50, -50, 90, 30 )
    time.sleep( 1.0 )
    
    miniDriver.setOutputs( 50, 50, 150, 150 )
    time.sleep( 1.0 )
    
    miniDriver.setOutputs( -50, -50, 90, 90 )
    time.sleep( 1.0 )

del miniDriver