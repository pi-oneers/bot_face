#! /usr/bin/env python

import os.path
import math
import time
import tornado.httpserver
#import tornado.websocket
import tornado.ioloop
import tornado.web
import sockjs.tornado
import multiprocessing
import mini_driver
import camera_streamer

JOYSTICK_DEAD_ZONE = 0.1
MAX_ABS_MOTOR_SPEED = 30.0
MAX_ABS_NECK_SPEED = 20.0    # Degrees per second

commandQueue = None
scriptPath = os.path.dirname( __file__ )
webPath = os.path.abspath( scriptPath + "/www" )

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
        
        self.commandQueue = multiprocessing.Queue()
        
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
    def update( self ):
        
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
class ConnectionHandler( sockjs.tornado.SockJSConnection ):
    
    #-----------------------------------------------------------------------------------------------
    def on_open( self, info ):
        print "Got cQ", commandQueue
        self.commandQueue = commandQueue
        
    #-----------------------------------------------------------------------------------------------
    def on_message( self, message ):
        
        #print "Got", message
        
        try:
            message = str( message )
        except Exception:
            print "failed"
            return
        
        if isinstance( message, str ):
            
            lineData = message.split( " " )
            if len( lineData ) > 0:
                
                if lineData[ 0 ] == "Centre":
                
                    self.commandQueue.put( [ "c" ] )
                
                elif lineData[ 0 ] == "StartStreaming":
                    
                    global cameraStreamer
                    cameraStreamer.startStreaming()
                
                elif lineData[ 0 ] == "Move" and len( lineData ) >= 3:
                    
                    motorJoystickX, motorJoystickY = \
                        self.extractNormalisedJoystickData( lineData[ 1 ], lineData[ 2 ] )
                        
                    # Set forward speed from motorJoystickY
                    leftMotorSpeed = MAX_ABS_MOTOR_SPEED*motorJoystickY
                    rightMotorSpeed = MAX_ABS_MOTOR_SPEED*motorJoystickY
                    
                    # Set turn speed from motorJoystickX
                    leftMotorSpeed += MAX_ABS_MOTOR_SPEED*motorJoystickX
                    rightMotorSpeed -= MAX_ABS_MOTOR_SPEED*motorJoystickX
                    
                    leftMotorSpeed = max( -MAX_ABS_MOTOR_SPEED, min( leftMotorSpeed, MAX_ABS_MOTOR_SPEED ) )
                    rightMotorSpeed = max( -MAX_ABS_MOTOR_SPEED, min( rightMotorSpeed, MAX_ABS_MOTOR_SPEED ) )
                    
                    print motorJoystickX, motorJoystickY, leftMotorSpeed, rightMotorSpeed
                    
                    self.commandQueue.put( [ "m", leftMotorSpeed, rightMotorSpeed ] )
                    
                elif lineData[ 0 ] == "PanTilt" and len( lineData ) >= 3:
                    
                    neckJoystickX, neckJoystickY = \
                        self.extractNormalisedJoystickData( lineData[ 1 ], lineData[ 2 ] )
                        
                    # Set pan and tilt angle speeds
                    panSpeed = -MAX_ABS_NECK_SPEED*neckJoystickX
                    tiltSpeed = -MAX_ABS_NECK_SPEED*neckJoystickY
                    
                    self.commandQueue.put( [ "l", panSpeed, tiltSpeed ] )

    #-----------------------------------------------------------------------------------------------
    def on_close(self):
        print 'connection closed'

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
robot = Robot()
cameraStreamer = camera_streamer.CameraStreamer()
commandQueue = robot.commandQueue

router = sockjs.tornado.SockJSRouter( 
    ConnectionHandler, '/robot_control' ) #, { "commandQueue" : robot.commandQueue } )
application = tornado.web.Application( [ 
    ( r"/", MainHandler ), 
    ( r"/css/(.*)", tornado.web.StaticFileHandler, { "path": webPath + "/css" } ),
    ( r"/fonts/(.*)", tornado.web.StaticFileHandler, { "path": webPath + "/fonts" } ),
    ( r"/js/(.*)", tornado.web.StaticFileHandler, { "path": webPath + "/js" } ) ] \
    + router.urls ) #,
    #template_path=webPath, static_path=webPath )
    
    #( r"/(.*)", tornado.web.StaticFileHandler, {"path": scriptPath + "/www" } ) ] \
 
#--------------------------------------------------------------------------------------------------- 
if __name__ == "__main__":
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen( 80 )
    
    robotPeriodicCallback = tornado.ioloop.PeriodicCallback( 
        robot.update, 100, io_loop=tornado.ioloop.IOLoop.instance() )
    robotPeriodicCallback.start()
    
    cameraStreamerPeriodicCallback = tornado.ioloop.PeriodicCallback( 
        cameraStreamer.update, 1000, io_loop=tornado.ioloop.IOLoop.instance() )
    cameraStreamerPeriodicCallback.start()
    
    tornado.ioloop.IOLoop.instance().start()