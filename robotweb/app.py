import math
import Pyro.core
from cgi import parse_qs

JOYSTICK_DEAD_ZONE = 0.2
LOG_FILENAME = "/home/robotweb/log.txt"

def application( environ, start_response ):

    #logFile = open( LOG_FILENAME, "a" )

    # Connect to Pyro
    robotControl = Pyro.core.getProxyForURI( "PYRONAME://robot_control_server" )

    #print >>logFile, "robotControl =", robotControl

    messageHandled = False
    parameters = parse_qs( environ['QUERY_STRING'] )

    #print >>logFile, "parameters =", parameters

    if 'motorJoystickX' in parameters and 'motorJoystickY' in parameters:
        
        motorJoystickX = 0.0
        motorJoystickY = 0.0
        
        try:
            motorJoystickX = float( parameters[ 'motorJoystickX' ][0] )
        except Exception:
            pass
        
        try:
            motorJoystickY = float( parameters[ 'motorJoystickY' ][0] )
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
        maxAbsMotorSpeed = robotControl.getMaxAbsMotorSpeed()
        leftMotorSpeed = maxAbsMotorSpeed*motorJoystickY
        rightMotorSpeed = maxAbsMotorSpeed*motorJoystickY
        
        # Set turn speed from motorJoystickX
        leftMotorSpeed += maxAbsMotorSpeed*motorJoystickX
        rightMotorSpeed -= maxAbsMotorSpeed*motorJoystickX
        
        #print >>logFile, "Setting motor speeds", leftMotorSpeed, rightMotorSpeed
        robotControl.setMotorSpeeds( leftMotorSpeed, rightMotorSpeed )
        messageHandled = True
     
     
    if messageHandled:
        status = '200 OK'
    else:
        status = '400 Bad Request'
    
    #logFile.close()

    response_headers = [('Content-type', 'text/plain'),
                        ('Content-Length', str(len(status)))]
    start_response(status, response_headers)
    return [status]
