
import logging
import math
import time
import Queue
import mini_driver

#--------------------------------------------------------------------------------------------------- 
class RobotController:
    
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