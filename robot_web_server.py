#! /usr/bin/env python

# Copyright (c) 2014, Dawn Robotics Ltd
# All rights reserved.

# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, 
# this list of conditions and the following disclaimer in the documentation 
# and/or other materials provided with the distribution.

# 3. Neither the name of the Dawn Robotics Ltd nor the names of its contributors 
# may be used to endorse or promote products derived from this software without 
# specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import logging

LOG_FILENAME = "/tmp/robot_web_server_log.txt"
logging.basicConfig( filename=LOG_FILENAME, level=logging.DEBUG )

# Also log to stdout
consoleHandler = logging.StreamHandler()
consoleHandler.setLevel( logging.DEBUG )
logging.getLogger( "" ).addHandler( consoleHandler )

import os.path
import math
import time
import signal
import tornado.httpserver
import tornado.ioloop
import tornado.web
import tornado.escape
import sockjs.tornado
import threading
import Queue
import camera_streamer
import robot_controller
import json
import ino_uploader
import subprocess

JOYSTICK_DEAD_ZONE = 0.1
MAX_ABS_MOTOR_SPEED = 80.0  # Duty cycle of motors (0 to 100%)
MAX_ABS_TURN_SPEED = 60.0   # Duty cycle of motors (0 to 100%)
MAX_ABS_NECK_SPEED = 30.0   # Degrees per second

robot = None

cameraStreamer = None
scriptPath = os.path.dirname( __file__ )
webPath = os.path.abspath( scriptPath + "/www" )
robotConnectionResultQueue = Queue.Queue()
isClosing = False

#--------------------------------------------------------------------------------------------------- 
def createRobot( resultQueue ):
    
    r = robot_controller.RobotController()
    resultQueue.put( r )
            
#--------------------------------------------------------------------------------------------------- 
class ConnectionHandler( sockjs.tornado.SockJSConnection ):
    
    #-----------------------------------------------------------------------------------------------
    def on_open( self, info ):
        
        pass
        
    #-----------------------------------------------------------------------------------------------
    def on_message( self, message ):
                
        try:
            message = str( message )
        except Exception:
            logging.warning( "Got a message that couldn't be converted to a string" )
            return
        
        if isinstance( message, str ):
            
            lineData = message.split( " " )
            if len( lineData ) > 0:
                
                if lineData[ 0 ] == "Centre":
                
                    if robot != None:
                        robot.commandQueue.put( [ "c" ] )
                
                elif lineData[ 0 ] == "StartStreaming":
                    
                    cameraStreamer.startStreaming()
                    
                elif lineData[ 0 ] == "Shutdown":
                    
                     result = subprocess.call( [ "poweroff" ] )
                     print "Result was", result
                    
                elif lineData[ 0 ] == "GetConfig":
                    
                    # Get the current configuration from the robot and return it
                    configDict = {}
                    
                    if robot != None:
                        configDict = robot.getConfigDict()
                    
                    self.send( json.dumps( configDict ) )
                
                elif lineData[ 0 ] == "SetConfig" and len( lineData ) >= 2:
                    
                    # Send the new configuration to the robot
                    configDict = {}
                    
                    print "Got config data", lineData[ 1 ]
                    
                    try:
                        configDict = json.loads( lineData[ 1 ] )
                    except Exception:
                        pass
                    
                    if robot != None:
                        robot.commandQueue.put( [ "s", configDict ] )
                
                elif lineData[ 0 ] == "GetLogs":
                    
                    # Return a dictionary containing the current logs
                    self.send( json.dumps( self.getLogsDict() ) )
                
                elif lineData[ 0 ] == "Move" and len( lineData ) >= 3:
                    
                    motorJoystickX, motorJoystickY = \
                        self.extractNormalisedJoystickData( lineData[ 1 ], lineData[ 2 ] )
                        
                    # Set forward speed from motorJoystickY
                    leftMotorSpeed = MAX_ABS_MOTOR_SPEED*motorJoystickY
                    rightMotorSpeed = MAX_ABS_MOTOR_SPEED*motorJoystickY
                    
                    # Set turn speed from motorJoystickX
                    leftMotorSpeed += MAX_ABS_TURN_SPEED*motorJoystickX
                    rightMotorSpeed -= MAX_ABS_TURN_SPEED*motorJoystickX
                    
                    leftMotorSpeed = max( -MAX_ABS_MOTOR_SPEED, min( leftMotorSpeed, MAX_ABS_MOTOR_SPEED ) )
                    rightMotorSpeed = max( -MAX_ABS_MOTOR_SPEED, min( rightMotorSpeed, MAX_ABS_MOTOR_SPEED ) )
                    
                    if robot != None:
                        robot.commandQueue.put( [ "m", leftMotorSpeed, rightMotorSpeed ] )
                    
                elif lineData[ 0 ] == "PanTilt" and len( lineData ) >= 3:
                    
                    neckJoystickX, neckJoystickY = \
                        self.extractNormalisedJoystickData( lineData[ 1 ], lineData[ 2 ] )
                        
                    # Set pan and tilt angle speeds
                    panSpeed = -MAX_ABS_NECK_SPEED*neckJoystickX
                    tiltSpeed = -MAX_ABS_NECK_SPEED*neckJoystickY
                    
                    if robot != None:
                        robot.commandQueue.put( [ "l", panSpeed, tiltSpeed ] )
                        
                elif lineData[ 0 ] == "SetNeckAngles" and len( lineData ) >= 3:
                    
                    panAngle = robot_controller.RobotController.CENTRE_ANGLE
                    tiltAngle = robot_controller.RobotController.CENTRE_ANGLE
                    
                    try:
                        panAngle = float( lineData[ 1 ] )
                    except Exception:
                        pass
                    
                    try:
                        tiltAngle = float( lineData[ 2 ] )
                    except Exception:
                        pass
                    
                    if robot != None:
                        robot.commandQueue.put( [ "n", panAngle, tiltAngle ] )

                    
    #-----------------------------------------------------------------------------------------------
    def on_close( self ):
        logging.info( "SockJS connection closed" )

    #-----------------------------------------------------------------------------------------------
    def getLogsDict( self ):
        
        logsDict = {}
        
        # Read in main logs file
        try:
            with open( LOG_FILENAME, "r" ) as logFile:
                logsDict[ "MainLog" ] = logFile.read()
        except Exception:
            pass
        
        # Read in Ino build output if it exists
        try:
            with open( ino_uploader.BUILD_OUTPUT_FILENAME, "r" ) as logFile:
                logsDict[ "InoBuildLog" ] = logFile.read()
        except Exception:
            pass

        return logsDict
        
    #-----------------------------------------------------------------------------------------------
    def extractNormalisedJoystickData( self, dataX, dataY ):
        
        joystickX = 0.0
        joystickY = 0.0
        
        try:
            joystickX = float( dataX )
        except Exception:
            pass
        
        try:
            joystickY = float( dataY )
        except Exception:
            pass

        stickVectorLength = math.sqrt( joystickX**2 + joystickY**2 )
        if stickVectorLength > 1.0:
            joystickX /= stickVectorLength
            joystickY /= stickVectorLength
        
        if stickVectorLength < JOYSTICK_DEAD_ZONE:
            joystickX = 0.0
            joystickY = 0.0
            
        return ( joystickX, joystickY )

#--------------------------------------------------------------------------------------------------- 
class MainHandler( tornado.web.RequestHandler ):
    
    #------------------------------------------------------------------------------------------------
    def get( self ):
        self.render( webPath + "/index.html" )
        
#--------------------------------------------------------------------------------------------------- 
def robotUpdate():
    
    global robot
    global isClosing
    
    if isClosing:
        tornado.ioloop.IOLoop.instance().stop()
        return
        
    if robot == None:
        
        if not robotConnectionResultQueue.empty():
            
            robot = robotConnectionResultQueue.get()
        
    else:
                
        robot.update()

#--------------------------------------------------------------------------------------------------- 
def sigintHandler( signum, frame ):
    global isClosing
    isClosing = True
        
#--------------------------------------------------------------------------------------------------- 
if __name__ == "__main__":
    
    signal.signal( signal.SIGINT, sigintHandler )
    
    # Create the configuration for the web server
    router = sockjs.tornado.SockJSRouter( 
        ConnectionHandler, '/robot_control' )
    application = tornado.web.Application( router.urls + [ 
        ( r"/", MainHandler ), 
        ( r"/(.*)", tornado.web.StaticFileHandler, { "path": webPath } ),
        ( r"/css/(.*)", tornado.web.StaticFileHandler, { "path": webPath + "/css" } ),
        ( r"/css/images/(.*)", tornado.web.StaticFileHandler, { "path": webPath + "/css/images" } ),
        ( r"/images/(.*)", tornado.web.StaticFileHandler, { "path": webPath + "/images" } ),
        ( r"/js/(.*)", tornado.web.StaticFileHandler, { "path": webPath + "/js" } ) ] )
    
    #( r"/(.*)", tornado.web.StaticFileHandler, {"path": scriptPath + "/www" } ) ] \
    
    # Create a camera streamer
    cameraStreamer = camera_streamer.CameraStreamer()
    
    # Start connecting to the robot asyncronously
    robotConnectionThread = threading.Thread( target=createRobot, 
        args=[ robotConnectionResultQueue ] )
    robotConnectionThread.start()

    # Now start the web server
    logging.info( "Starting web server..." )
    http_server = tornado.httpserver.HTTPServer( application )
    http_server.listen( 80 )
    
    robotPeriodicCallback = tornado.ioloop.PeriodicCallback( 
        robotUpdate, 100, io_loop=tornado.ioloop.IOLoop.instance() )
    robotPeriodicCallback.start()
    
    cameraStreamerPeriodicCallback = tornado.ioloop.PeriodicCallback( 
        cameraStreamer.update, 1000, io_loop=tornado.ioloop.IOLoop.instance() )
    cameraStreamerPeriodicCallback.start()
    
    tornado.ioloop.IOLoop.instance().start()
    
    # Shut down code
    robotConnectionThread.join()
    
    if robot != None:
        robot.disconnect()
    else:
        if not robotConnectionResultQueue.empty():
            robot = robotConnectionResultQueue.get()
            robot.disconnect()