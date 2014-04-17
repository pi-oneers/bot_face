#! /usr/bin/python

import re
import subprocess
import time

ipRegex = re.compile( "inet addr\:(?P<ipAddress>[0-9\.]*)" )

while True:

    ifconfigData = subprocess.check_output( [ "ifconfig", "wlan0" ] )
    searchResults = ipRegex.search( ifconfigData )

    if searchResults == None:

        print "No ip address found"
        subprocess.call( [ "sudo", "ifdown", "wlan0" ] )
        subprocess.call( [ "sudo", "service", "hostapd", "stop" ] )
        subprocess.call( [ "sudo", "service", "isc-dhcp-server", "stop" ] )

        time.sleep( 2.0 )
        
        subprocess.call( [ "sudo", "ifup", "wlan0" ] )
        
        time.sleep( 2.0 )
        
        subprocess.call( [ "sudo", "service", "hostapd", "start" ] )
        subprocess.call( [ "sudo", "service", "isc-dhcp-server", "start" ] )
        
        with open( "/tmp/keep_wifi_alive.log", "w+" ) as logFile:
            print "Revived wifi"

    time.sleep( 1.0 )
