import math
import Pyro.core
from cgi import parse_qs

JOYSTICK_DEAD_ZONE = 0.2

def application( environ, start_response ):

    # Connect to Pyro
    robotControl = Pyro.core.getProxyForURI( "PYRONAME://robot_control_server" )

    messageHandled = False
    parameters = parse_qs( environ['QUERY_STRING'] )

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
        
        if motorJoystickX < JOYSTICK_DEAD_ZONE:
            motorJoystickX = 0.0
        if motorJoystickY < JOYSTICK_DEAD_ZONE:
            motorJoystickY = 0.0
            
        # Set forward speed from motorJoystickY
        maxAbsMotorSpeed = robotControl.getMaxAbsMotorSpeed()
        leftMotorSpeed = maxAbsMotorSpeed*motorJoystickY
        rightMotorSpeed = maxAbsMotorSpeed*motorJoystickY
        
        # Set turn speed from motorJoystickX
        leftMotorSpeed += maxAbsMotorSpeed*motorJoystickX
        rightMotorSpeed -= maxAbsMotorSpeed*motorJoystickX
        
        robotControl.setMotorSpeeds( leftMotorSpeed, rightMotorSpeed )
        messageHandled = True
     
     
    if messageHandled:
        status = '200 OK'
    else:
        status = '400 Bad Request'
    
    response_headers = [('Content-type', 'text/plain'),
                        ('Content-Length', str(len(status)))]
    start_response(status, response_headers)
    return [status]