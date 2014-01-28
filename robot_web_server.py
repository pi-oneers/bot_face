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
import mini_driver
import camera_streamer

JOYSTICK_DEAD_ZONE = 0.1
MAX_ABS_MOTOR_SPEED = 50.0  # Duty cycle of motors (0 to 100%)
MAX_ABS_TURN_SPEED = 30.0   # Duty cycle of motors (0 to 100%)
MAX_ABS_NECK_SPEED = 20.0   # Degrees per second

robot = None

cameraStreamer = None
scriptPath = os.path.dirname( __file__ )
webPath = os.path.abspath( scriptPath + "/www" )
robotConnectionResultQueue = Queue.Queue()
isClosing = False

#--------------------------------------------------------------------------------------------------- 
class Robot:
    
    MIN_ANGLE = 0.0
    MAX_ANGLE = 180.0
    CENTRE_ANGLE = (MIN_ANGLE + MAX_ANGLE)/2.0
    
    MAX_TIME_DIFF = 0.25
    TIME_BETWEEN_SERVO_SETTING_UPDATES = 1.0
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self ):
        
        self.miniDriver = mini_driver.MiniDriver()
        connected = self.miniDriver.connect()
        if not connected:
            raise Exception( "Unable to connect to the mini driver" )
        
        self.commandQueue = Queue.Queue()
        
        self.panPulseWidthMin = 700
        self.panPulseWidthMax = 2100
        self.tiltPulseWidthMin = 550
        self.tiltPulseWidthMax = 2400
        
        self.leftMotorSpeed = 0
        self.rightMotorSpeed = 0
        self.panAngle = self.CENTRE_ANGLE
        self.tiltAngle = self.CENTRE_ANGLE
        
        self.panSpeed = 0.0
        self.tiltSpeed = 0.0
        
        self.lastServoSettingsSendTime = 0.0
        self.lastUpdateTime = 0.0
    
    #-----------------------------------------------------------------------------------------------
    def __del__( self ):
        
        self.disconnect()
    
    #-----------------------------------------------------------------------------------------------
    def disconnect( self ):
        
        self.miniDriver.disconnect()
    
    #-----------------------------------------------------------------------------------------------
    def update( self ):
        
        if not self.miniDriver.isConnected():
            return
        
        curTime = time.time()
        timeDiff = min( curTime - self.lastUpdateTime, self.MAX_TIME_DIFF )
        
        # Pull commands from the commnd queue
        while not self.commandQueue.empty():
            
            command = self.commandQueue.get_nowait()
                
            if command[ 0 ] == "m":     # Move
                
                self.leftMotorSpeed = command[ 1 ]
                self.rightMotorSpeed = command[ 2 ]
            
            elif command[ 0 ] == "l":   # Look
            
                self.panSpeed = command[ 1 ]
                self.tiltSpeed = command[ 2 ]
                
            elif command[ 0 ] == "c":   # Centre
            
                self.panAngle = self.CENTRE_ANGLE
                self.tiltAngle = self.CENTRE_ANGLE
                
            elif command[ 0 ] == "n":   # Set neck angles
            
                self.panAngle = command[ 1 ]
                self.tiltAngle = command[ 2 ]
        
        # Update the pan and tilt angles
        self.panAngle += self.panSpeed*timeDiff
        self.tiltAngle += self.tiltSpeed*timeDiff
        
        self.panAngle = max( self.MIN_ANGLE, min( self.panAngle, self.MAX_ANGLE ) )
        self.tiltAngle = max( self.MIN_ANGLE, min( self.tiltAngle, self.MAX_ANGLE ) )
        
        # Update the mini driver
        self.miniDriver.setOutputs(
            self.leftMotorSpeed, self.rightMotorSpeed, self.panAngle, self.tiltAngle )
        
        self.lastUpdateTime = curTime
        
        # Send servo settings if needed
        if curTime - self.lastServoSettingsSendTime >= self.TIME_BETWEEN_SERVO_SETTING_UPDATES:
            
            self.miniDriver.setPanServoLimits( self.panPulseWidthMin, self.panPulseWidthMax )
            self.miniDriver.setTiltServoLimits( self.tiltPulseWidthMin, self.tiltPulseWidthMax )
 
            self.lastServoSettingsSendTime = curTime

#--------------------------------------------------------------------------------------------------- 
def createRobot( resultQueue ):
    
    r = Robot()
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
                    
                elif lineData[ 0 ] == "GetConfig":
                    
                    config = {
                        "panMinPWM" : 500,
                        "panMaxPWM" : 2400,
                        "tiltMinPWM" : 500,
                        "tiltMaxPWM" : 2400,
                        "batteryType" : "6xNiMh"
                    }
                    self.send( tornado.escape.json_encode( config ) )
                
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
                    
                    panAngle = Robot.CENTRE_ANGLE
                    tiltAngle = Robot.CENTRE_ANGLE
                    
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
    def on_close(self):
        logging.info( "SockJS connection closed" )

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