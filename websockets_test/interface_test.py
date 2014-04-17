#! /usr/bin/python

import sockjs_client
import time

client = sockjs_client.Client( "/robot_control", "localhost" )  # Replace "localhost" with the IP address of the robot to run remotely

client.connect()
time.sleep( 0.1 )   # Need a bit of time to establish connection...

try:
    while True:
        client.send( "Move 0.0 1.0" )    # Straight ahead
        time.sleep( 1.0 )
        client.send( "Move 1.0 0.0" )    # Turn right
        time.sleep( 1.0 )

        client.send( "Move 0.0 0.0" )
        client.send( "PanTilt -1.0 0.0" )    # Look left
        time.sleep( 1.0 )

        client.send( "PanTilt 0.0 0.0" )
        client.send( "Centre" )    # Recentre
        
except KeyboardInterrupt:   # Catch Ctrl+C
    print "Caught Ctrl+C"

print "Shutting down"
client.send( "Move 0.0 0.0" )
time.sleep( 0.1 )
client.send( "PanTilt 0.0 0.0" )
client.send( "Centre" )
time.sleep( 0.1 )

client.stop()
client.join()