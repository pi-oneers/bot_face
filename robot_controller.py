
import logging
import os.path
import math
import time
import Queue
import mini_driver
import json

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
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self ):
        
        self.scriptPath = os.path.dirname( __file__ )
        
        self.miniDriver = mini_driver.MiniDriver()
        connected = self.miniDriver.connect()
        if not connected:
            raise Exception( "Unable to connect to the mini driver" )
        
        self.commandQueue = Queue.Queue()
        
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
                
            elif command[ 0 ] == "s":   # Set config values
            
                configDict = command[ 1 ]
                self.readDataFromConfigDict( configDict )
                self.writeConfigFile()
        
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