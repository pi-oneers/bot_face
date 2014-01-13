import os.path
import re
import serial
import multiprocessing
import time

import ino_uploader

MESSAGE_MARKER = chr( 0xFF ) + chr( 0xFF )
COMMAND_ID_READ_DB_ENTRY = 1
COMMAND_ID_WRITE_DB_ENTRY = 2

COMMAND_ID_GET_FIRMWARE_INFO = 1
COMMAND_ID_SET_OUTPUTS = 2

RESPONSE_ID_FIRMWARE_INFO = 1
RESPONSE_ID_INVALID_COMMAND = 2
RESPONSE_ID_INVALID_CHECK_SUM = 3

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
class SerialReadProcess( multiprocessing.Process ):

    #-----------------------------------------------------------------------------------------------
    def __init__( self, serialPort, responseQueue ):
        
        multiprocessing.Process.__init__( self )
        self.serialPort = serialPort
        self.responseQueue = responseQueue
        self.serialBuffer = ""
        
        self.stopEvent = multiprocessing.Event()

    #-----------------------------------------------------------------------------------------------
    def stop( self ):
        self.stopEvent.set()

    #-----------------------------------------------------------------------------------------------
    def isStopped( self ):
        return self.stopEvent.is_set()
    
    #-----------------------------------------------------------------------------------------------
    def run( self ):
        
        while not self.isStopped():
            
            numBytesAvailable = self.serialPort.inWaiting()
            if numBytesAvailable > 0:
                
                newBytes = self.serialPort.read( numBytesAvailable )
                self.serialBuffer += newBytes

                print "---> Got", newBytes
                
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
                    
    #-----------------------------------------------------------------------------------------------
    def processMessage( self, msgBuffer ):
        
        #print "Got message with {0} bytes".format( len( msgBuffer ) )
        
        if calculateCheckSum( msgBuffer ) != ord( msgBuffer[ -1 ] ):
            print "Warning: Got message with invalid checksum"
            self.responseQueue.put( "Invalid" )
            return
        
        messageId = ord( msgBuffer[ 2 ] )
        if messageId == RESPONSE_ID_FIRMWARE_INFO:
            
            dataBytes = msgBuffer[ 4:-1 ]
            if len( dataBytes ) < 4:
                
                print "Warning: Got message with invalid number of bytes"
                self.responseQueue.put( "Invalid" )
                
            else:
                
                idHigh = ord( dataBytes[ 0 ] )
                idLow = ord( dataBytes[ 1 ] )
                versionMajor = ord( dataBytes[ 2 ] )
                versionMinor = ord( dataBytes[ 3 ] )
                
                firmwareInfo = FirmwareInfo( idHigh, idLow, versionMajor, versionMinor )
                #print "Read ", firmwareInfo
                
                self.responseQueue.put( firmwareInfo )
            
        elif messageId == RESPONSE_ID_INVALID_COMMAND:
            
            print "Invalid command sent"
            self.responseQueue.put( "Invalid" )
            
        elif messageId == RESPONSE_ID_INVALID_CHECK_SUM:
            
            print "Sent message had invalid checksum"
            self.responseQueue.put( "Invalid" )
        
        else:
            
            print "Error: Got unrecognised response id -", messageId
            self.responseQueue.put( "Invalid" )

#---------------------------------------------------------------------------------------------------
class Connection():
    
    STARTUP_DELAY = 10.0     # Needed to wait for mini driver reset
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, serialPortName, baudRate ):
        
        self.serialPort = serial.Serial( serialPortName, baudRate, timeout=0 )
        
        self.responseQueue = multiprocessing.Queue()
    
        self.serialReadProcess = SerialReadProcess( self.serialPort, self.responseQueue )
        self.serialReadProcess.start()
        
        time.sleep( self.STARTUP_DELAY )
        
    #-----------------------------------------------------------------------------------------------
    def __del__( self ):
        
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
    
#---------------------------------------------------------------------------------------------------
class MiniDriver():
    
    SERIAL_PORT_NAME = "/dev/ttyUSB0"
    BAUD_RATE = 57600
    FIRMWARE_MAIN_FILENAME = "mini_driver_firmware/mini_driver_firmware.ino"
    BOARD_MODEL = "atmega8"
    
    EXPECTED_FIRMWARE_INFO = FirmwareInfo( 0xAC, 0xED, 0, 1 )
    
    MAX_ABS_MOTOR_SPEED = 100
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self ):
        
        self.scriptPath = os.path.dirname( __file__ )
        self.connection = None
        
    #-----------------------------------------------------------------------------------------------
    def connect( self ):
        
        """Establishes a connection with the Mini Driver and confirms that it contains
           the correct version of the firmware. If not then the routine builds the
           firmware using Ino and uploads it to the Mini Driver"""
        
        self.connection = Connection( self.SERIAL_PORT_NAME, self.BAUD_RATE )
        firmwareInfo = self.connection.getFirmwareInfo()
        print firmwareInfo
        print self.__getExpectedFirmwareInfo()
        print firmwareInfo == self.__getExpectedFirmwareInfo()
        
        if firmwareInfo != self.__getExpectedFirmwareInfo():
            
            del self.connection
            self.connection = None
            
            print "Unable to connect to correct firmware, uploading..."
            uploadResult = ino_uploader.upload( self.__getFirmwareDir(), 
                serialPortName=self.SERIAL_PORT_NAME, boardModel=self.BOARD_MODEL )
            
            if uploadResult == True:
                
                self.connection = Connection( self.SERIAL_PORT_NAME, self.BAUD_RATE )
                firmwareInfo = self.connection.getFirmwareInfo()
                print firmwareInfo
                
                if firmwareInfo != self.__getExpectedFirmwareInfo():
            
                    del self.connection
                    self.connection = None
            
        return self.isConnected()
    
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
        
        leftMotorSpeed = max( -self.MAX_ABS_MOTOR_SPEED, min( leftMotorSpeed, self.MAX_ABS_MOTOR_SPEED ) )
        leftMotorSpeed += SIGNED_TO_UNSIGNED_OFFSET
        rightMotorSpeed = max( -self.MAX_ABS_MOTOR_SPEED, min( rightMotorSpeed, self.MAX_ABS_MOTOR_SPEED ) )
        rightMotorSpeed += SIGNED_TO_UNSIGNED_OFFSET
        
        panAngle = max( 0, min( panAngle, 180 ) )
        tiltAngle = max( 0, min( tiltAngle, 180 ) )
        
        if self.connection != None:
            self.connection.setOutputs( leftMotorSpeed, rightMotorSpeed, panAngle, tiltAngle )
    
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