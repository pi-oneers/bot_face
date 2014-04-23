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
import os.path
import re
import serial
import threading
import time
import Queue

import ino_uploader

MESSAGE_MARKER = chr( 0xFF ) + chr( 0xFF )
COMMAND_ID_READ_DB_ENTRY = 1
COMMAND_ID_WRITE_DB_ENTRY = 2

COMMAND_ID_GET_FIRMWARE_INFO = 1
COMMAND_ID_SET_OUTPUTS = 2
COMMAND_ID_SET_PAN_SERVO_LIMITS = 3
COMMAND_ID_SET_TILT_SERVO_LIMITS = 4

RESPONSE_ID_FIRMWARE_INFO = 1
RESPONSE_ID_INVALID_COMMAND = 2
RESPONSE_ID_INVALID_CHECK_SUM = 3
RESPONSE_ID_BATTERY_READING = 4

ADC_REF_VOLTAGE = 5.0
BATTERY_VOLTAGE_SCALE = 2.0   # Battery voltage is divided by 2 before it is 
                              # passed to the ADC so we must undo that

#---------------------------------------------------------------------------------------------------
class FirmwareInfo:
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, idHigh=0x00, idLow=0x00, versionMajor=0, versionMinor=0 ):
        
        self.idHigh = idHigh
        self.idLow = idLow
        self.versionMajor = versionMajor
        self.versionMinor = versionMinor
        
    #-----------------------------------------------------------------------------------------------
    def __repr__( self ):
        
        return "{0} {1}.{2}".format( 
            hex( self.idHigh << 8 | self.idLow ).upper(), self.versionMajor, self.versionMinor )
            
    #-----------------------------------------------------------------------------------------------
    def __eq__( self, other ): 
        return self.__dict__ == other.__dict__
        
    #-----------------------------------------------------------------------------------------------
    def __ne__( self, other ): 
        return self.__dict__ != other.__dict__

        
#---------------------------------------------------------------------------------------------------
def calculateCheckSum( msgBuffer ):
    
    # Use all of the data apart from the message start bytes
    checkSum = 0
    
    for msgByte in msgBuffer[ 2:-1 ]:
        checkSum += ord( msgByte )

    return ~checkSum & 0xFF

#---------------------------------------------------------------------------------------------------
class SerialReadProcess( threading.Thread ):

    DEFAULT_UPDATE_RATE_HZ = 100.0

    #-----------------------------------------------------------------------------------------------
    def __init__( self, serialPort, responseQueue, updateRateHz=DEFAULT_UPDATE_RATE_HZ ):
        
        threading.Thread.__init__( self )
        self.serialPort = serialPort
        self.responseQueue = responseQueue
        self.serialBuffer = ""
        
        self.stopEvent = threading.Event()
        
        self.updateRateHz = updateRateHz
        if self.updateRateHz <= 0.0:
            self.updateRateHz = 1.0

    #-----------------------------------------------------------------------------------------------
    def stop( self ):
        self.stopEvent.set()

    #-----------------------------------------------------------------------------------------------
    def isStopped( self ):
        return self.stopEvent.is_set()
    
    #-----------------------------------------------------------------------------------------------
    def run( self ):
        
        while not self.isStopped():
            
            startTime = time.time()
            
            numBytesAvailable = self.serialPort.inWaiting()
            if numBytesAvailable > 0:
                
                newBytes = self.serialPort.read( numBytesAvailable )
                self.serialBuffer += newBytes
               
                # Check to see if we've received a message
                msgStartPos = self.serialBuffer.find( MESSAGE_MARKER )
                while msgStartPos != -1:
                    
                    msgFound = False
                    
                    self.serialBuffer = self.serialBuffer[ msgStartPos: ]
                    bufferLength = len( self.serialBuffer )
                    if bufferLength > 4:
                        
                        msgLength = ord( self.serialBuffer[ 3 ] )
                        if msgLength <= bufferLength:
                            
                            # Once we've got all the bytes for a message, process them and
                            # check the rest of a buffer for another message
                            self.processMessage( self.serialBuffer[ :msgLength ] )
                            self.serialBuffer = self.serialBuffer[ msgLength: ]
                            
                            msgFound = True
                    
                    if msgFound:
                        msgStartPos = self.serialBuffer.find( MESSAGE_MARKER )
                    else:
                        msgStartPos = -1
                        
            endTime = time.time()
            sleepTime = 1.0/self.updateRateHz - (endTime - startTime)
            if sleepTime > 0.0:
                time.sleep( sleepTime )
                    
    #-----------------------------------------------------------------------------------------------
    def processMessage( self, msgBuffer ):
          
        if calculateCheckSum( msgBuffer ) != ord( msgBuffer[ -1 ] ):
            logging.warning( "Got message with invalid checksum" )
            self.responseQueue.put( "Invalid" )
            return
        
        messageId = ord( msgBuffer[ 2 ] )
        if messageId == RESPONSE_ID_FIRMWARE_INFO:
            
            dataBytes = msgBuffer[ 4:-1 ]
            if len( dataBytes ) < 4:
                
                logging.warning( "Got message with invalid number of bytes" )
                self.responseQueue.put( "Invalid" )
                
            else:
                
                idHigh = ord( dataBytes[ 0 ] )
                idLow = ord( dataBytes[ 1 ] )
                versionMajor = ord( dataBytes[ 2 ] )
                versionMinor = ord( dataBytes[ 3 ] )
                
                firmwareInfo = FirmwareInfo( idHigh, idLow, versionMajor, versionMinor )
 
                self.responseQueue.put( firmwareInfo )
            
        elif messageId == RESPONSE_ID_INVALID_COMMAND:
            
            logging.info( "Invalid command sent" )
            self.responseQueue.put( "Invalid" )
            
        elif messageId == RESPONSE_ID_INVALID_CHECK_SUM:
            
            logging.info( "Sent message had invalid checksum" )
            self.responseQueue.put( "Invalid" )
        
        elif messageId == RESPONSE_ID_BATTERY_READING:
            
            dataBytes = msgBuffer[ 4:-1 ]
            if len( dataBytes ) < 2:
                
                logging.warning( "Got message with invalid number of bytes" )
                self.responseQueue.put( "Invalid" )
                
            else:
                batteryReading = ord( dataBytes[ 0 ] ) << 8 | ord( dataBytes[ 1 ] )
                print "Battery voltage is", BATTERY_VOLTAGE_SCALE * ADC_REF_VOLTAGE * float( batteryReading )/1023.0
        
        else:
            
            logging.warning( "Got unrecognised response id - " + str( messageId ) )
            self.responseQueue.put( "Invalid" )

#---------------------------------------------------------------------------------------------------
class Connection():
    
    STARTUP_DELAY = 10.0     # Needed to wait for mini driver reset
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, serialPortName, baudRate ):
        
        self.serialPort = serial.Serial( serialPortName, baudRate, timeout=0 )
        
        self.responseQueue = Queue.Queue()
        self.serialReadProcess = SerialReadProcess( self.serialPort, self.responseQueue )
        self.serialReadProcess.start()
        
        time.sleep( self.STARTUP_DELAY )
        
    #-----------------------------------------------------------------------------------------------
    def __del__( self ):
        
        self.close()
        
    #-----------------------------------------------------------------------------------------------
    def close( self ):
        
        if self.serialReadProcess != None:
            self.serialReadProcess.stop()
            self.serialReadProcess.join()
            self.serialReadProcess = None
        
    #-----------------------------------------------------------------------------------------------
    def getFirmwareInfo( self ):
        
        msgBuffer = MESSAGE_MARKER + chr( COMMAND_ID_GET_FIRMWARE_INFO ) + chr( 5 ) + chr( 0 )
        msgBuffer = msgBuffer[ :-1 ] + chr( calculateCheckSum( msgBuffer ) )
        
        self.serialPort.write( msgBuffer )
        
        #for b in msgBuffer:
        #    print "Sent", ord( b )
        
        response = None
        try:
            response = self.responseQueue.get( timeout=5.0 )
        except Exception:
            pass
        
        if not isinstance( response, FirmwareInfo ):
            response = FirmwareInfo()
        
        return response   
        
    #-----------------------------------------------------------------------------------------------
    def setOutputs( self, leftMotorSpeed, rightMotorSpeed, panAngle, tiltAngle ):
        
        msgBuffer = MESSAGE_MARKER + chr( COMMAND_ID_SET_OUTPUTS ) \
            + chr( 9 ) \
            + chr( leftMotorSpeed ) + chr( rightMotorSpeed ) \
            + chr( panAngle ) + chr( tiltAngle ) \
            + chr( 0 )
        msgBuffer = msgBuffer[ :-1 ] + chr( calculateCheckSum( msgBuffer ) )
        
        self.serialPort.write( msgBuffer )
        
    #-----------------------------------------------------------------------------------------------
    def setPanServoLimits( self, servoMin, servoMax ):
        
        msgBuffer = MESSAGE_MARKER + chr( COMMAND_ID_SET_PAN_SERVO_LIMITS) \
            + chr( 9 ) \
            + chr( (servoMin >> 8) & 0xFF ) + chr( servoMin & 0xFF ) \
            + chr( (servoMax >> 8) & 0xFF ) + chr( servoMax & 0xFF ) \
            + chr( 0 )
        msgBuffer = msgBuffer[ :-1 ] + chr( calculateCheckSum( msgBuffer ) )

        self.serialPort.write( msgBuffer )
        
    #-----------------------------------------------------------------------------------------------
    def setTiltServoLimits( self, servoMin, servoMax ):
        
        msgBuffer = MESSAGE_MARKER + chr( COMMAND_ID_SET_TILT_SERVO_LIMITS) \
            + chr( 9 ) \
            + chr( (servoMin >> 8) & 0xFF ) + chr( servoMin & 0xFF ) \
            + chr( (servoMax >> 8) & 0xFF ) + chr( servoMax & 0xFF ) \
            + chr( 0 )
        msgBuffer = msgBuffer[ :-1 ] + chr( calculateCheckSum( msgBuffer ) )

        self.serialPort.write( msgBuffer )
    
#---------------------------------------------------------------------------------------------------
class MiniDriver():
    
    SERIAL_PORT_NAME = "/dev/ttyUSB0"
    BAUD_RATE = 57600
    FIRMWARE_MAIN_FILENAME = "mini_driver_firmware/mini_driver_firmware.ino"
    BOARD_MODEL = "atmega8"
    
    EXPECTED_FIRMWARE_INFO = FirmwareInfo( 0xAC, 0xED, 0, 1 )
    
    MAX_ABS_MOTOR_SPEED = 100
    MIN_PULSE_WIDTH = 200
    MAX_PULSE_WIDTH = 2800
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self ):
        
        self.scriptPath = os.path.dirname( __file__ )
        self.connection = None
    
    #-----------------------------------------------------------------------------------------------
    def __del__( self ):
        
        self.disconnect()
    
    #-----------------------------------------------------------------------------------------------
    def connect( self ):
        
        """Establishes a connection with the Mini Driver and confirms that it contains
           the correct version of the firmware. If not then the routine builds the
           firmware using Ino and uploads it to the Mini Driver"""
        
        self.connection = Connection( self.SERIAL_PORT_NAME, self.BAUD_RATE )
        firmwareInfo = self.connection.getFirmwareInfo()
        logging.info( "Read " + str( firmwareInfo ) )
        logging.info( "Expected " + str( self.__getExpectedFirmwareInfo() ) )
        
        if firmwareInfo != self.__getExpectedFirmwareInfo():
            
            self.connection.close()
            self.connection = None
            
            logging.info( "Unable to connect to correct firmware, uploading..." )
            uploadResult = ino_uploader.upload( self.__getFirmwareDir(), 
                serialPortName=self.SERIAL_PORT_NAME, boardModel=self.BOARD_MODEL )
            
            if uploadResult == True:
                
                self.connection = Connection( self.SERIAL_PORT_NAME, self.BAUD_RATE )
                firmwareInfo = self.connection.getFirmwareInfo()
                logging.info( "Read " + str( firmwareInfo ) )
                
                if firmwareInfo != self.__getExpectedFirmwareInfo():
            
                    self.connection.close()
                    self.connection = None
            
        return self.isConnected()
    
    #-----------------------------------------------------------------------------------------------
    def disconnect( self ):
        
        if self.isConnected():
            self.connection.close()
            self.connection = None
    
    #-----------------------------------------------------------------------------------------------
    def isConnected( self ):
        
        return self.connection != None
    
    #-----------------------------------------------------------------------------------------------
    def getFirmwareInfo( self ):
        
        result = FirmwareInfo()
        
        if self.connection != None:
            result = self.connection.getFirmwareInfo()
    
        return result
    
    #-----------------------------------------------------------------------------------------------
    def setOutputs( self, leftMotorSpeed, rightMotorSpeed, panAngle, tiltAngle ):
        
        SIGNED_TO_UNSIGNED_OFFSET = 128
        
        leftMotorSpeed = max( -self.MAX_ABS_MOTOR_SPEED, min( int( leftMotorSpeed ), self.MAX_ABS_MOTOR_SPEED ) )
        leftMotorSpeed += SIGNED_TO_UNSIGNED_OFFSET
        rightMotorSpeed = max( -self.MAX_ABS_MOTOR_SPEED, min( int( rightMotorSpeed ), self.MAX_ABS_MOTOR_SPEED ) )
        rightMotorSpeed += SIGNED_TO_UNSIGNED_OFFSET
        
        panAngle = max( 0, min( int( panAngle ), 180 ) )
        tiltAngle = max( 0, min( int( tiltAngle ), 180 ) )
        
        if self.connection != None:
            self.connection.setOutputs( leftMotorSpeed, rightMotorSpeed, panAngle, tiltAngle )
            
    #-----------------------------------------------------------------------------------------------
    def setPanServoLimits( self, servoMin, servoMax ):
     
        servoMin = max( self.MIN_PULSE_WIDTH, min( int( servoMin ), self.MAX_PULSE_WIDTH ) )
        servoMax = max( self.MIN_PULSE_WIDTH, min( int( servoMax ), self.MAX_PULSE_WIDTH ) )
        
        if self.connection != None:
            self.connection.setPanServoLimits( servoMin, servoMax )
            
    #-----------------------------------------------------------------------------------------------
    def setTiltServoLimits( self, servoMin, servoMax ):
     
        servoMin = max( self.MIN_PULSE_WIDTH, min( int( servoMin ), self.MAX_PULSE_WIDTH ) )
        servoMax = max( self.MIN_PULSE_WIDTH, min( int( servoMax ), self.MAX_PULSE_WIDTH ) )
        
        if self.connection != None:
            self.connection.setTiltServoLimits( servoMin, servoMax )
    
    #-----------------------------------------------------------------------------------------------
    def __getFirmwareDir( self ):
        
        firmwarePath = self.scriptPath + "/" + self.FIRMWARE_MAIN_FILENAME
        return os.path.dirname( firmwarePath )
    
    #-----------------------------------------------------------------------------------------------
    def __getExpectedFirmwareInfo( self ):
 
        idLow = None
        idHigh = None
        versionMajor = None
        versionMinor = None
        
        # Open up the firmware source file and search for the firmware Id
        firmwarePath = self.scriptPath + "/" + self.FIRMWARE_MAIN_FILENAME
        firmwareFile = open( firmwarePath )
        
        idRegEx = re.compile( "const[ ]*uint16_t[ ]*FIRMWARE_ID[ ]*=[ ]*(?P<firmwareId>\w*?)[ ]*;" )
        versionMajorRegEx = re.compile( "const[ ]*uint8_t[ ]*VERSION_MAJOR[ ]*=[ ]*(?P<versionMajor>\w*?)[ ]*;" )
        versionMinorRegEx = re.compile( "const[ ]*uint8_t[ ]*VERSION_MINOR[ ]*=[ ]*(?P<versionMinor>\w*?)[ ]*;" )
        
        for line in firmwareFile:
            
            match = idRegEx.search( line )
            if match != None:
                
                try:
                    firmwareId = int( match.groupdict()[ "firmwareId" ], 16 )
                    idHigh = (firmwareId >> 8) & 0xFF
                    idLow = firmwareId & 0xFF
                    
                except ValueError:
                    pass
                
            match = versionMajorRegEx.search( line )
            if match != None:
                
                try:
                    versionMajor = int( match.groupdict()[ "versionMajor" ] )
                    
                except ValueError:
                    pass
                
            match = versionMinorRegEx.search( line )
            if match != None:
                
                try:
                    versionMinor = int( match.groupdict()[ "versionMinor" ] )
                    
                except ValueError:
                    pass
        
        firmwareFile.close()
           
        if idHigh == None or idLow == None:
            
            raise Exception( "Unable to find expected firmware Id" )
        
        if versionMajor == None or versionMinor == None:
            
            raise Exception( "Unable to find expected firmware version" )
        
        return FirmwareInfo( idHigh, idLow, versionMajor, versionMinor )
