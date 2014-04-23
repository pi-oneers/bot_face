/* Copyright (c) 2014, Dawn Robotics Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution.

3. Neither the name of the Dawn Robotics Ltd nor the names of its contributors 
may be used to endorse or promote products derived from this software without 
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include <Servo.h>

//------------------------------------------------------------------------------
const uint8_t VERSION_MAJOR = 0;
const uint8_t VERSION_MINOR = 29;
const uint16_t FIRMWARE_ID = 0xACED;

const uint16_t MAX_MSG_SIZE = 16;
const uint16_t MSG_START_BYTES = 0xFFFF;
const uint16_t MSG_ID_POS = 2;
const uint16_t MSG_SIZE_POS = 3;
const uint16_t MSG_HEADER_SIZE = 2 + 1 + 1; // Start bytes + Id + size

const uint8_t COMMAND_ID_GET_FIRMWARE_INFO = 1;
const uint8_t COMMAND_ID_SET_OUTPUTS = 2;
const uint8_t COMMAND_ID_SET_PAN_SERVO_LIMITS = 3;
const uint8_t COMMAND_ID_SET_TILT_SERVO_LIMITS = 4;

const uint8_t RESPONSE_ID_FIRMWARE_INFO = 1;
const uint8_t RESPONSE_ID_INVALID_COMMAND = 2;
const uint8_t RESPONSE_ID_INVALID_CHECK_SUM = 3;
const uint8_t RESPONSE_ID_BATTERY_READING = 4;

const int LEFT_MOTOR_DIR_PIN = 7;
const int LEFT_MOTOR_PWM_PIN = 9;
const int RIGHT_MOTOR_DIR_PIN = 8;
const int RIGHT_MOTOR_PWM_PIN = 10;
const int PAN_SERVO_PIN = 4;
const int TILT_SERVO_PIN = 3;
const int BATTERY_VOLTAGE_PIN = A7;
                                            
const unsigned long BATTERY_VOLTAGE_READ_MS = 10;
                                            
const int ABSOLUTE_MIN_PWM = 400;
const int ABSOLUTE_MAX_PWM = 2600;

const unsigned long MOTOR_COMMAND_TIMEOUT_MS = 2000;

//------------------------------------------------------------------------------
// This class is provided because the Arduino Servo library maps minimum and 
// maximum bounds to a single byte for some reason.
class ServoLimits
{
    // MIN_PULSE_WIDTH and MAX_PULSE_WIDTH come from Servo.h
    public: ServoLimits( int minPWM=MIN_PULSE_WIDTH, int maxPWM=MAX_PULSE_WIDTH )
    {
        setLimits( minPWM, maxPWM );
    }
    
    public: void setLimits( int minPWM, int maxPWM )
    {
        mMinPWM = constrain( minPWM, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM );
        mMaxPWM = constrain( maxPWM, ABSOLUTE_MIN_PWM, ABSOLUTE_MAX_PWM );
    }
    
    public: int convertAngleToPWM( int angle )
    {
        angle = constrain( angle, 0, 180 );
        return map( angle, 0, 180, mMinPWM, mMaxPWM );
    }
    
    public: int getMinPWM() const { return mMinPWM; }
    public: int getMaxPWM() const { return mMaxPWM; }
    
    private: int mMinPWM;
    private: int mMaxPWM;
};

//------------------------------------------------------------------------------
enum eMotorDirection
{
    eMD_Forwards,
    eMD_Backwards
};

enum eMessageState
{
    eMS_WaitingForMessage,
    eMS_ReceivingMessage
};

eMessageState gMessageState = eMS_WaitingForMessage;
uint8_t gMsgBuffer[ MAX_MSG_SIZE ];
uint16_t gNumMsgBytesReceived = 0;

Servo gPanServo;
ServoLimits gPanServoLimits;
Servo gTiltServo;
ServoLimits gTiltServoLimits;

uint8_t gLeftMotorDutyCycle = 0;
uint8_t gRightMotorDutyCycle = 0;
eMotorDirection gLeftMotorDirection = eMD_Forwards;
eMotorDirection gRightMotorDirection = eMD_Forwards;
uint8_t gPanServoAngle = 90;
uint8_t gTiltServoAngle = 90;
unsigned long gLastCommandTime = 0;
unsigned long gLastBatteryVoltageTime = 0;

const uint8_t NUM_TICKS_PER_MOTOR_WAVE = 100;
volatile uint8_t gCurMotorWaveTick = 0;

//------------------------------------------------------------------------------
uint8_t getMessageId() { return gMsgBuffer[ MSG_ID_POS ]; }
uint8_t getMessageSize() { return gMsgBuffer[ MSG_SIZE_POS ]; }
void receiveMessages();
void processMessage();
void sendFirmwareInfoResponse();
void sendInvalidCommandResponse();
void sendInvalidChecksumResponse();
void sendBatteryReadingMessage( int batteryReading );
uint8_t calculateCheckSum( const uint8_t* pData, uint8_t msgSize );

//------------------------------------------------------------------------------
void setup()
{
    // Set up timer 2 to generate an interrupt 62500 times a second
    noInterrupts();
    TCCR2 = 2; //1;        // Activate time, no prescaling of timer source
    
    TIMSK |= (1 << TOIE2);    // Activate timer 2 overflow interrupt
    
    
    interrupts();
    
    gPanServo.attach( PAN_SERVO_PIN );
    gTiltServo.attach( TILT_SERVO_PIN );
    
    pinMode( LEFT_MOTOR_DIR_PIN, OUTPUT );
    pinMode( LEFT_MOTOR_PWM_PIN, OUTPUT );
    pinMode( RIGHT_MOTOR_DIR_PIN, OUTPUT );
    pinMode( RIGHT_MOTOR_PWM_PIN, OUTPUT );

    Serial.begin( 57600 );
}

//------------------------------------------------------------------------------
void loop()
{
    // Read any commands from the serial connection
    receiveMessages();
    
    // Turn off the motors if we haven't received a command for a while
    unsigned long curTime = millis();
    
    if ( curTime - gLastCommandTime > MOTOR_COMMAND_TIMEOUT_MS )
    {
        gLeftMotorDutyCycle = 0;
        gRightMotorDutyCycle = 0;
        gLeftMotorDirection = eMD_Forwards;
        gRightMotorDirection = eMD_Forwards;
    }
    
    // Update the motor directions and servo angles
    digitalWrite( LEFT_MOTOR_DIR_PIN, ( eMD_Forwards == gLeftMotorDirection ? HIGH : LOW ) );
    digitalWrite( RIGHT_MOTOR_DIR_PIN, ( eMD_Forwards == gRightMotorDirection ? HIGH : LOW ) );
    
    gPanServo.writeMicroseconds( gPanServoLimits.convertAngleToPWM( gPanServoAngle ) );
    gTiltServo.writeMicroseconds( gTiltServoLimits.convertAngleToPWM( gTiltServoAngle ) );
    
    if ( curTime - gLastBatteryVoltageTime >= BATTERY_VOLTAGE_READ_MS )
    {
        // Read in the battery voltage
        sendBatteryReadingMessage( analogRead( BATTERY_VOLTAGE_PIN ) );
        gLastBatteryVoltageTime = curTime;
    }
}

//------------------------------------------------------------------------------
void receiveMessages()
{
    bool bMessageReceived = false;
    int numBytesAvailable = Serial.available();
    
    while ( numBytesAvailable > 0 && !bMessageReceived )
    {
        switch ( gMessageState )
        {
            case eMS_WaitingForMessage:
            {
                int numBytesToRead = max( min( numBytesAvailable, MSG_HEADER_SIZE - gNumMsgBytesReceived ), 0 );
                int numBytesRead = Serial.readBytes( (char*)&gMsgBuffer[ gNumMsgBytesReceived ], numBytesToRead );
                gNumMsgBytesReceived += numBytesRead;
                                
                if ( MSG_HEADER_SIZE == gNumMsgBytesReceived )
                {
                    if ( MSG_START_BYTES == *((uint16_t*)gMsgBuffer) )
                    {
                        // We have a message header
                        gMessageState = eMS_ReceivingMessage;
                    }
                    else
                    {
                        // Discard the first byte as it is not part of a message
                        gMsgBuffer[ 0 ] = gMsgBuffer[ 1 ];
                        gMsgBuffer[ 1 ] = gMsgBuffer[ 2 ];
                        gMsgBuffer[ 2 ] = gMsgBuffer[ 3 ];
                        gNumMsgBytesReceived = 3;
                    }
                }
                
                break;
            }
            case eMS_ReceivingMessage:
            {
                int numBytesToRead = max( min( numBytesAvailable, getMessageSize() - gNumMsgBytesReceived ), 0 );
                int numBytesRead = Serial.readBytes( (char*)&gMsgBuffer[ gNumMsgBytesReceived ], numBytesToRead );
                gNumMsgBytesReceived += numBytesRead;
                
                if ( getMessageSize() == gNumMsgBytesReceived )
                {
                    processMessage();
                    bMessageReceived = true;
                    
                    // Prepare for next message
                    gNumMsgBytesReceived = 0;
                    gMessageState = eMS_WaitingForMessage;
                }
                
                break;
            }
            default:
            {
                // We should never get here, but just in case, return to eMS_WaitingForMessage
                gNumMsgBytesReceived = 0;
                gMessageState = eMS_WaitingForMessage;
            }
        }
        
        numBytesAvailable = Serial.available();
    }
}

//------------------------------------------------------------------------------
void processMessage()
{
    // Check the checksum of the message
    uint8_t calculatedCheckSum = calculateCheckSum( gMsgBuffer, getMessageSize() );
    
    if ( calculatedCheckSum != gMsgBuffer[ getMessageSize() - 1 ] )
    {
        sendInvalidCheckSumResponse();
    }
    
    // Handle the command Id
    bool bCommandHandled = false;
    switch ( getMessageId() )
    {
        case COMMAND_ID_GET_FIRMWARE_INFO:
        {
            sendFirmwareInfoResponse();
            bCommandHandled = true;
            
            break;
        }
        case COMMAND_ID_SET_OUTPUTS:
        {
            if ( getMessageSize() == 9 )
            {
                int leftMotorSpeed = (int)gMsgBuffer[ 4 ] - 128;
                int rightMotorSpeed = (int)gMsgBuffer[ 5 ] - 128;
                gPanServoAngle = constrain( gMsgBuffer[ 6 ], 0, 180 );
                gTiltServoAngle = constrain( gMsgBuffer[ 7 ], 0, 180 );
            
                gLeftMotorDirection = ( leftMotorSpeed >= 0 ? eMD_Forwards : eMD_Backwards );
                gRightMotorDirection = ( rightMotorSpeed >= 0 ? eMD_Forwards : eMD_Backwards );
                gLeftMotorDutyCycle = constrain( abs( leftMotorSpeed ), 0, 100 );
                gRightMotorDutyCycle = constrain( abs( rightMotorSpeed ), 0, 100 );
                gLastCommandTime = millis();
            
                bCommandHandled = true;
            }
        
            break;
        }
        case COMMAND_ID_SET_PAN_SERVO_LIMITS:
        case COMMAND_ID_SET_TILT_SERVO_LIMITS:
        {
            if ( getMessageSize() == 9 )
            {
                uint16_t servoMin = gMsgBuffer[ 4 ] << 8 | gMsgBuffer[ 5 ];
                uint16_t servoMax = gMsgBuffer[ 6 ] << 8 | gMsgBuffer[ 7 ];
                
                if ( getMessageId() == COMMAND_ID_SET_PAN_SERVO_LIMITS )
                {                   
                    gPanServoLimits.setLimits( servoMin, servoMax );
                }
                else
                {
                    gTiltServoLimits.setLimits( servoMin, servoMax );
                }
                
                bCommandHandled = true;
            }
            break;
        }
    }
    
    if ( !bCommandHandled )
    {
        sendInvalidCommandResponse();
    }
}

//------------------------------------------------------------------------------
void sendFirmwareInfoResponse()
{
    uint8_t msgData[] = 
    {
        0xFF, 0xFF, RESPONSE_ID_FIRMWARE_INFO, 0,   // Header
        FIRMWARE_ID >> 8,  FIRMWARE_ID & 0xFF, 
        VERSION_MAJOR, VERSION_MINOR, 0
    };
    
    const uint8_t MSG_SIZE = sizeof( msgData );
    msgData[ 3 ] = MSG_SIZE;
    msgData[ MSG_SIZE - 1 ] = calculateCheckSum( msgData, MSG_SIZE );
    
    Serial.write( msgData, MSG_SIZE );
}

//------------------------------------------------------------------------------
void sendInvalidCommandResponse()
{
    uint8_t msgData[] = 
    {
        0xFF, 0xFF, RESPONSE_ID_INVALID_COMMAND, 0, 0
    };
    
    const uint8_t MSG_SIZE = sizeof( msgData );
    msgData[ 3 ] = MSG_SIZE;
    msgData[ MSG_SIZE - 1 ] = calculateCheckSum( msgData, MSG_SIZE );
    
    Serial.write( msgData, MSG_SIZE );
}

//------------------------------------------------------------------------------
void sendInvalidCheckSumResponse()
{
    uint8_t msgData[] = 
    {
        0xFF, 0xFF, RESPONSE_ID_INVALID_CHECK_SUM, 0, 0
    };
    
    const uint8_t MSG_SIZE = sizeof( msgData );
    msgData[ 3 ] = MSG_SIZE;
    msgData[ MSG_SIZE - 1 ] = calculateCheckSum( msgData, MSG_SIZE );
    
    Serial.write( msgData, MSG_SIZE );
}

//------------------------------------------------------------------------------
void sendBatteryReadingMessage( int batteryReading )
{    
    uint8_t msgData[] = 
    {
        0xFF, 0xFF, RESPONSE_ID_BATTERY_READING, 0, // Header
       (batteryReading >> 8) & 0xFF, batteryReading & 0xFF, 0
    };
    
    const uint8_t MSG_SIZE = sizeof( msgData );
    msgData[ 3 ] = MSG_SIZE;
    msgData[ MSG_SIZE - 1 ] = calculateCheckSum( msgData, MSG_SIZE );
    
    Serial.write( msgData, MSG_SIZE );
}

//------------------------------------------------------------------------------
uint8_t calculateCheckSum( const uint8_t* pData, uint8_t msgSize )
{
    uint32_t sum = 0;
    
    // Use all of the data apart from the message start bytes and the byte
    // that will store the checksum
    for ( uint8_t i = 2; i < msgSize - 1; i++ )
    {
        sum += pData[ i ];
    }
    
    return (uint8_t)(~sum);
}

//------------------------------------------------------------------------------
ISR( TIMER2_OVF_vect )        // Interrupt service routine for motor PWM
{
    gCurMotorWaveTick++;
    
    digitalWrite( LEFT_MOTOR_PWM_PIN, ( gCurMotorWaveTick > gLeftMotorDutyCycle ? LOW : HIGH ) );
    digitalWrite( RIGHT_MOTOR_PWM_PIN, ( gCurMotorWaveTick > gRightMotorDutyCycle ? LOW : HIGH ) );
    
    if ( gCurMotorWaveTick >= 100 )
    {
        gCurMotorWaveTick = 0;
    }
}

