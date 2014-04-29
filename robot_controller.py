
import logging
import os.path
import math
import time
import Queue
import mini_driver
import json
import threading

LEFT_MOTOR_SCALE = 0.9

#--------------------------------------------------------------------------------------------------- 
class RobotController:
    
    MIN_ANGLE = 0.0
    MAX_ANGLE = 180.0
    CENTRE_ANGLE = (MIN_ANGLE + MAX_ANGLE)/2.0
    
    MAX_TIME_DIFF = 0.25
    TIME_BETWEEN_SERVO_SETTING_UPDATES = 1.0
    
    CONFIG_DIR = "config"
    CONFIG_FILENAME = "config.json"
    
    MIN_PULSE_WIDTH = 400
    MAX_PULSE_WIDTH = 2600
    
    JOYSTICK_DEAD_ZONE = 0.1
    MAX_ABS_MOTOR_SPEED = 80.0  # Duty cycle of motors (0 to 100%)
    MAX_ABS_TURN_SPEED = 60.0   # Duty cycle of motors (0 to 100%)
    MAX_ABS_NECK_SPEED = 30.0   # Degrees per second
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self ):
        
        self.scriptPath = os.path.dirname( __file__ )
        
        self.miniDriver = mini_driver.MiniDriver()
        connected = self.miniDriver.connect()
        if not connected:
            raise Exception( "Unable to connect to the mini driver" )
          
        self.panPulseWidthMin = 700
        self.panPulseWidthMax = 2100
        self.tiltPulseWidthMin = 550
        self.tiltPulseWidthMax = 2400
        
        self.tryToLoadConfigFile()
        
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
    def tryToLoadConfigFile( self ):
        
        absConfigFilename = os.path.abspath( 
            self.scriptPath + "/" + self.CONFIG_DIR + "/" + self.CONFIG_FILENAME )
        
        if os.path.exists( absConfigFilename ):
            try:
                with open( absConfigFilename ) as configFile:
                    
                    configDict = json.load( configFile )
                    self.readDataFromConfigDict( configDict )
                    
            except Exception as e:
                logging.warning( "Unable to load config file. Exception was " + str( e ) )
    
    #-----------------------------------------------------------------------------------------------
    def readDataFromConfigDict( self, configDict ):
        
        if "panPulseWidthMin" in configDict:
            self.panPulseWidthMin = self.parsePulseWidth( 
                configDict[ "panPulseWidthMin" ], self.panPulseWidthMin )
        
        if "panPulseWidthMax" in configDict:
            self.panPulseWidthMax = self.parsePulseWidth( 
                configDict[ "panPulseWidthMax" ], self.panPulseWidthMax )
        
        if "tiltPulseWidthMin" in configDict:
            self.tiltPulseWidthMin = self.parsePulseWidth( 
                configDict[ "tiltPulseWidthMin" ], self.tiltPulseWidthMin )
        
        if "tiltPulseWidthMax" in configDict:
            self.tiltPulseWidthMax = self.parsePulseWidth( 
                configDict[ "tiltPulseWidthMax" ], self.tiltPulseWidthMax )
    
    #-----------------------------------------------------------------------------------------------
    def parsePulseWidth( self, inputData, defaultValue=MIN_PULSE_WIDTH ):
        
        result = defaultValue
        
        try:
            result = int( inputData )
        except Exception:
            pass
        
        return max( self.MIN_PULSE_WIDTH, min( result, self.MAX_PULSE_WIDTH ) )
    
    #-----------------------------------------------------------------------------------------------
    def writeConfigFile( self ):
        
        configDict = self.getConfigDict()
        
        absConfigDir = os.path.abspath( self.scriptPath + "/" + self.CONFIG_DIR )
        absConfigFilename = absConfigDir + "/" + self.CONFIG_FILENAME
        
        if not os.path.exists( absConfigDir ):
            os.makedirs( absConfigDir )
            
        with open( absConfigFilename, "w" ) as configFile:
            
            json.dump( configDict, configFile )
        
    #-----------------------------------------------------------------------------------------------
    def getConfigDict( self ):    
    
        configDict = {
            "panPulseWidthMin" : self.panPulseWidthMin,
            "panPulseWidthMax" : self.panPulseWidthMax,
            "tiltPulseWidthMin" : self.tiltPulseWidthMin,
            "tiltPulseWidthMax" : self.tiltPulseWidthMax
        }
        
        return configDict
    
    #-----------------------------------------------------------------------------------------------
    def setConfigDict( self, configDict ):
        
        self.readDataFromConfigDict( configDict )
        self.writeConfigFile()
    
    #-----------------------------------------------------------------------------------------------
    def getStatusDict( self ):
        
        statusDict = {
            "batteryVoltage" : self.miniDriver.getBatteryVoltage()
        }
        
        return statusDict
    
    #-----------------------------------------------------------------------------------------------
    def normaliseJoystickData( self, joystickX, joystickY ):
        
        stickVectorLength = math.sqrt( joystickX**2 + joystickY**2 )
        if stickVectorLength > 1.0:
            joystickX /= stickVectorLength
            joystickY /= stickVectorLength
        
        if stickVectorLength < self.JOYSTICK_DEAD_ZONE:
            joystickX = 0.0
            joystickY = 0.0
            
        return ( joystickX, joystickY )
    
    #-----------------------------------------------------------------------------------------------
    def centreNeck( self ):
        
        self.panAngle = self.CENTRE_ANGLE
        self.tiltAngle = self.CENTRE_ANGLE
        self.panSpeed = 0.0
        self.tiltSpeed = 0.0
    
    #-----------------------------------------------------------------------------------------------
    def setMotorJoystickPos( self, joystickX, joystickY ):
        
        joystickX, joystickY = self.normaliseJoystickData( joystickX, joystickY )
        
        # Set forward speed from joystickY
        leftMotorSpeed = self.MAX_ABS_MOTOR_SPEED*joystickY
        rightMotorSpeed = self.MAX_ABS_MOTOR_SPEED*joystickY
        
        # Set turn speed from joystickX
        leftMotorSpeed += self.MAX_ABS_TURN_SPEED*joystickX
        rightMotorSpeed -= self.MAX_ABS_TURN_SPEED*joystickX
        
        leftMotorSpeed = max( -self.MAX_ABS_MOTOR_SPEED, min( leftMotorSpeed, self.MAX_ABS_MOTOR_SPEED ) )
        rightMotorSpeed = max( -self.MAX_ABS_MOTOR_SPEED, min( rightMotorSpeed, self.MAX_ABS_MOTOR_SPEED ) )
        
        self.leftMotorSpeed = leftMotorSpeed*LEFT_MOTOR_SCALE
        self.rightMotorSpeed = rightMotorSpeed
    
    #-----------------------------------------------------------------------------------------------
    def setNeckJoystickPos( self, joystickX, joystickY ):
        
        joystickX, joystickY = self.normaliseJoystickData( joystickX, joystickY )
        
        # Set pan and tilt angle speeds
        self.panSpeed = -self.MAX_ABS_NECK_SPEED*joystickX
        self.tiltSpeed = -self.MAX_ABS_NECK_SPEED*joystickY
    
    #-----------------------------------------------------------------------------------------------
    def setNeckAngles( self, panAngle, tiltAngle ):
        
        self.panAngle = max( self.MIN_ANGLE, min( panAngle, self.MAX_ANGLE ) )
        self.tiltAngle = max( self.MIN_ANGLE, min( tiltAngle, self.MAX_ANGLE ) )
    
    #-----------------------------------------------------------------------------------------------
    def update( self ):
        
        if not self.miniDriver.isConnected():
            return
        
        curTime = time.time()
        timeDiff = min( curTime - self.lastUpdateTime, self.MAX_TIME_DIFF )
        
        # Update the pan and tilt angles
        self.panAngle += self.panSpeed*timeDiff
        self.tiltAngle += self.tiltSpeed*timeDiff
        
        self.panAngle = max( self.MIN_ANGLE, min( self.panAngle, self.MAX_ANGLE ) )
        self.tiltAngle = max( self.MIN_ANGLE, min( self.tiltAngle, self.MAX_ANGLE ) )
        
        # Update the mini driver
        self.miniDriver.setOutputs(
            self.leftMotorSpeed, self.rightMotorSpeed, self.panAngle, self.tiltAngle )
        self.miniDriver.update()
        
        # Send servo settings if needed
        if curTime - self.lastServoSettingsSendTime >= self.TIME_BETWEEN_SERVO_SETTING_UPDATES:
            
            self.miniDriver.setPanServoLimits( self.panPulseWidthMin, self.panPulseWidthMax )
            self.miniDriver.setTiltServoLimits( self.tiltPulseWidthMin, self.tiltPulseWidthMax )
 
            self.lastServoSettingsSendTime = curTime
            
        self.lastUpdateTime = curTime