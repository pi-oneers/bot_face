#! /usr/bin/env python

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
import math
import time
import Queue
import mini_driver
import threading

#--------------------------------------------------------------------------------------------------- 
class RobotController:
    
    MIN_ANGLE = 0.0
    MAX_ANGLE = 180.0
    CENTRE_ANGLE = (MIN_ANGLE + MAX_ANGLE)/2.0
    
    MAX_UPDATE_TIME_DIFF = 0.25
    TIME_BETWEEN_SERVO_SETTING_UPDATES = 1.0
    
    JOYSTICK_DEAD_ZONE = 0.1
    MAX_ABS_NECK_SPEED = 30.0   # Degrees per second
    
    MOTION_COMMAND_TIMEOUT = 2.0 # If no commands for the motors are recieved in this time then
                                 # the motors (drive and servo) are set to zero speed
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, robotConfig ):
        
        self.miniDriver = mini_driver.MiniDriver()
        connected = self.miniDriver.connect()
        if not connected:
            raise Exception( "Unable to connect to the mini driver" )
        
        self.robotConfig = robotConfig
        self.leftMotorSpeed = 0
        self.rightMotorSpeed = 0
        self.panAngle = self.CENTRE_ANGLE
        self.tiltAngle = self.CENTRE_ANGLE
        
        self.panSpeed = 0.0
        self.tiltSpeed = 0.0
        
        self.lastServoSettingsSendTime = 0.0
        self.lastUpdateTime = 0.0
        self.lastMotionCommandTime = time.time()
    
    #-----------------------------------------------------------------------------------------------
    def __del__( self ):
        
        self.disconnect()
    
    #-----------------------------------------------------------------------------------------------
    def disconnect( self ):
        
        self.miniDriver.disconnect()
    
    #-----------------------------------------------------------------------------------------------
    def getStatusDict( self ):
        
        presetMaxAbsMotorSpeed, presetMaxAbsTurnSpeed = self.miniDriver.getPresetMotorSpeeds()
        
        statusDict = {
            "batteryVoltage" : self.miniDriver.getBatteryVoltage(),
            "presetMaxAbsMotorSpeed" : presetMaxAbsMotorSpeed,
            "presetMaxAbsTurnSpeed" : presetMaxAbsTurnSpeed
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
        
        if self.robotConfig.usePresetMotorSpeeds:
            
            maxAbsMotorSpeed, maxAbsTurnSpeed = self.miniDriver.getPresetMotorSpeeds()
            
        else:
            
            maxAbsMotorSpeed = self.robotConfig.customMaxAbsMotorSpeed
            maxAbsTurnSpeed = self.robotConfig.customMaxAbsTurnSpeed
        
        # Set forward speed from joystickY
        leftMotorSpeed = maxAbsMotorSpeed*joystickY
        rightMotorSpeed = maxAbsMotorSpeed*joystickY
        
        # Set turn speed from joystickX
        leftMotorSpeed += maxAbsTurnSpeed*joystickX
        rightMotorSpeed -= maxAbsTurnSpeed*joystickX
        
        leftMotorSpeed = max( -maxAbsMotorSpeed, min( leftMotorSpeed, maxAbsMotorSpeed ) )
        rightMotorSpeed = max( -maxAbsMotorSpeed, min( rightMotorSpeed, maxAbsMotorSpeed ) )
        
        self.leftMotorSpeed = leftMotorSpeed*self.robotConfig.leftMotorScale
        self.rightMotorSpeed = rightMotorSpeed
        
        self.lastMotionCommandTime = time.time()
    
    #-----------------------------------------------------------------------------------------------
    def setMotorSpeeds( self, leftMotorSpeed, rightMotorSpeed ):
        
        if self.robotConfig.usePresetMotorSpeeds:
            
            maxAbsMotorSpeed, maxAbsTurnSpeed = self.miniDriver.getPresetMotorSpeeds()
            
        else:
            
            maxAbsMotorSpeed = self.robotConfig.customMaxAbsMotorSpeed
            maxAbsTurnSpeed = self.robotConfig.customMaxAbsTurnSpeed
        
        self.leftMotorSpeed = max( -maxAbsMotorSpeed, min( leftMotorSpeed, maxAbsMotorSpeed ) )
        self.rightMotorSpeed = max( -maxAbsMotorSpeed, min( rightMotorSpeed, maxAbsMotorSpeed ) )
        
        self.lastMotionCommandTime = time.time()
    
    #-----------------------------------------------------------------------------------------------
    def setNeckJoystickPos( self, joystickX, joystickY ):
        
        joystickX, joystickY = self.normaliseJoystickData( joystickX, joystickY )
        
        # Set pan and tilt angle speeds
        self.panSpeed = -self.MAX_ABS_NECK_SPEED*joystickX
        self.tiltSpeed = -self.MAX_ABS_NECK_SPEED*joystickY
        
        self.lastMotionCommandTime = time.time()
    
    #-----------------------------------------------------------------------------------------------
    def setNeckAngles( self, panAngle, tiltAngle ):
        
        self.panAngle = max( self.MIN_ANGLE, min( panAngle, self.MAX_ANGLE ) )
        self.tiltAngle = max( self.MIN_ANGLE, min( tiltAngle, self.MAX_ANGLE ) )
        self.panSpeed = 0.0
        self.tiltSpeed = 0.0
        
        self.lastMotionCommandTime = time.time()
    
    #-----------------------------------------------------------------------------------------------
    def update( self ):
        
        if not self.miniDriver.isConnected():
            return
        
        curTime = time.time()
        timeDiff = min( curTime - self.lastUpdateTime, self.MAX_UPDATE_TIME_DIFF )
        
        # Turn off the motors if we haven't received a motion command for a while
        if curTime - self.lastMotionCommandTime > self.MOTION_COMMAND_TIMEOUT:

            self.leftMotorSpeed = 0.0
            self.rightMotorSpeed = 0.0
            self.panSpeed = 0.0
            self.tiltSpeed = 0.0
        
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
            
            self.miniDriver.setPanServoLimits( 
                self.robotConfig.panPulseWidthMin, 
                self.robotConfig.panPulseWidthMax )
            self.miniDriver.setTiltServoLimits( 
                self.robotConfig.tiltPulseWidthMin, 
                self.robotConfig.tiltPulseWidthMax )
 
            self.lastServoSettingsSendTime = curTime
            
        self.lastUpdateTime = curTime