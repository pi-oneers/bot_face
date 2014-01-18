#! /usr/bin/env python

import math
import tornado.httpserver
#import tornado.websocket
import tornado.ioloop
import tornado.web
import sockjs.tornado
import multiprocessing
import mini_driver

JOYSTICK_DEAD_ZONE = 0.1
MAX_ABS_MOTOR_SPEED = 30.0

commandQueue = None

#--------------------------------------------------------------------------------------------------- 
class Robot:
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self ):
        
        self.miniDriver = mini_driver.MiniDriver()
        connected = self.miniDriver.connect()
        if not connected:
            raise Exception( "Unable to connect to the mini driver" )
        
        self.commandQueue = multiprocessing.Queue()
        
        self.leftMotorSpeed = 0
        self.rightMotorSpeed = 0
        self.panAngle = 60
        self.tiltAngle = 90
        
    #-----------------------------------------------------------------------------------------------
    def update( self ):
        
        # Pull commands from the commnd queue
        while not self.commandQueue.empty():
            
            command = self.commandQueue.get_nowait()
                
            if command[ 0 ] == "m":
                
                self.leftMotorSpeed = command[ 1 ]
                self.rightMotorSpeed = command[ 2 ]
                
        # Update the mini driver
        #print "Sending...", self.rightMotorSpeed, self.panAngle, self.tiltAngle
        
        self.miniDriver.setOutputs(
            self.leftMotorSpeed, self.rightMotorSpeed, self.panAngle, self.tiltAngle )
 
#--------------------------------------------------------------------------------------------------- 
class ConnectionHandler( sockjs.tornado.SockJSConnection ):
    
    #-----------------------------------------------------------------------------------------------
    #def initialize( self, commandQueue ):
    #    print "Got cQ", commandQueue
    #    self.commandQueue = commandQueue
    
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
                if lineData[ 0 ] == "Move":
                    
                    motorJoystickX = 0.0
                    motorJoystickY = 0.0
                    
                    try:
                        motorJoystickX = float( lineData[ 1 ] )
                    except Exception:
                        pass
                    
                    try:
                        motorJoystickY = float( lineData[ 2 ] )
                    except Exception:
                        pass

                    stickVectorLength = math.sqrt( motorJoystickX**2 + motorJoystickY**2 )
                    if stickVectorLength > 1.0:
                        motorJoystickX /= stickVectorLength
                        motorJoystickY /= stickVectorLength
                    
                    if abs( motorJoystickX ) < JOYSTICK_DEAD_ZONE:
                        motorJoystickX = 0.0
                    if abs( motorJoystickY ) < JOYSTICK_DEAD_ZONE:
                        motorJoystickY = 0.0
                        
                    # Set forward speed from motorJoystickY
                    leftMotorSpeed = MAX_ABS_MOTOR_SPEED*motorJoystickY
                    rightMotorSpeed = MAX_ABS_MOTOR_SPEED*motorJoystickY
                    
                    # Set turn speed from motorJoystickX
                    leftMotorSpeed += MAX_ABS_MOTOR_SPEED*motorJoystickX
                    rightMotorSpeed -= MAX_ABS_MOTOR_SPEED*motorJoystickX
                    
                    #print "Putting command", "m", leftMotorSpeed, rightMotorSpeed
                    self.commandQueue.put( [ "m", leftMotorSpeed, rightMotorSpeed ] )

    #-----------------------------------------------------------------------------------------------
    def on_close(self):
        print 'connection closed'
 
#--------------------------------------------------------------------------------------------------- 
robot = Robot()
commandQueue = robot.commandQueue

router = sockjs.tornado.SockJSRouter( 
    ConnectionHandler, '/robot_control' ) #, { "commandQueue" : robot.commandQueue } )
application = tornado.web.Application( router.urls )
 
#--------------------------------------------------------------------------------------------------- 
if __name__ == "__main__":
    http_server = tornado.httpserver.HTTPServer(application)
    http_server.listen( 8081 )
    
    periodicCallback = tornado.ioloop.PeriodicCallback( 
        robot.update, 100, io_loop=tornado.ioloop.IOLoop.instance() )
    periodicCallback.start()
    tornado.ioloop.IOLoop.instance().start()