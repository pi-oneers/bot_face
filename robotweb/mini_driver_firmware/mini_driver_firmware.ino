
#include <Servo.h>

//------------------------------------------------------------------------------
const uint8_t VERSION_MAJOR = 0;
const uint8_t VERSION_MINOR = 25;
const uint16_t FIRMWARE_ID = 0xACED;

const uint16_t MAX_MSG_SIZE = 16;
const uint16_t MSG_START_BYTES = 0xFFFF;
const uint16_t MSG_ID_POS = 2;
const uint16_t MSG_SIZE_POS = 3;
const uint16_t MSG_HEADER_SIZE = 2 + 1 + 1; // Start bytes + Id + size

const uint8_t COMMAND_ID_GET_FIRMWARE_INFO = 1;
const uint8_t COMMAND_ID_SET_OUTPUTS = 2;

const uint8_t RESPONSE_ID_FIRMWARE_INFO = 1;
const uint8_t RESPONSE_ID_INVALID_COMMAND = 2;
const uint8_t RESPONSE_ID_INVALID_CHECK_SUM = 3;

const int LEFT_MOTOR_DIR_PIN = 7;
const int LEFT_MOTOR_PWM_PIN = 9;
const int RIGHT_MOTOR_DIR_PIN = 8;
const int RIGHT_MOTOR_PWM_PIN = 10;
const int PAN_SERVO_PIN = 4;
const int TILT_SERVO_PIN = 3;

const unsigned long MOTOR_COMMAND_TIMEOUT_MS = 2000;

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
Servo gTiltServo;

uint8_t gLeftMotorDutyCycle = 0;
uint8_t gRightMotorDutyCycle = 0;
eMotorDirection gLeftMotorDirection = eMD_Forwards;
eMotorDirection gRightMotorDirection = eMD_Forwards;
uint8_t gPanServoAngle = 90;
uint8_t gTiltServoAngle = 90;
unsigned long gLastCommandTime = 0;

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
    
    gPanServo.write( gPanServoAngle );
    gTiltServo.write( gTiltServoAngle );
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
                gRightMotorDutyCycle = constrain( abs( leftMotorSpeed ), 0, 100 );
                gLastCommandTime = millis();
            
                Serial.println( gPanServoAngle );
            
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

