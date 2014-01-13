#!/usr/bin/env python

import time
import os
import Pyro.core
import Pyro.naming
import multiprocessing
import mini_driver

#---------------------------------------------------------------------------------------------------
class UpdateProcess( multiprocessing.Process ):

    #-----------------------------------------------------------------------------------------------
    def __init__( self, robotControl, updateRateHz=50.0 ):
        
        multiprocessing.Process.__init__( self )
        self.robotControl = robotControl
        self.updateRateHz = updateRateHz
        if self.updateRateHz <= 0.0:
            self.updateRateHz = 1.0
        
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
            
            loopStartTime = time.time()
            
            self.robotControl.miniDriver.setOutputs(
                self.robotControl.leftMotorSpeed,
                self.robotControl.rightMotorSpeed,
                self.robotControl.panAngle,
                self.robotControl.tiltAngle )
                
            maxLoopTime = 1.0/self.updateRateHz
            sleepTime = maxLoopTime - (time.time() - loopStartTime)
            if sleepTime > 0.0:
                time.sleep( 0.0 )

#---------------------------------------------------------------------------------------------------
class RobotControl( Pyro.core.ObjBase ):

    MAX_ABS_MOTOR_SPEED = 50

    #-----------------------------------------------------------------------------------------------
    def __init__( self ):
        Pyro.core.ObjBase.__init__( self )
        
        self.miniDriver = mini_driver.MiniDriver()
        connected = self.miniDriver.connect()
        if not connected:
            raise Exception( "Unable to connect to the mini driver" )
        
        self.leftMotorSpeed = 0
        self.rightMotorSpeed = 0
        self.panAngle = 90
        self.tiltAngle = 90
        
        self.updateProcess = UpdateProcess( self )
        self.updateProcess.start()
    
    #-----------------------------------------------------------------------------------------------
    def __del__( self ):
        
        if self.updateProcess != None:
            self.updateProcess.stop()
            self.updateProcess.join()
            self.updateProcess = None
        
        if self.miniDriver != None:
            del self.miniDriver
            self.miniDriver = None
    
    #-----------------------------------------------------------------------------------------------
    def setMotorSpeeds( self, leftMotorSpeed, rightMotorSpeed ):       
    
        absLimit = self.getMaxAbsMotorSpeed()
        self.leftMotorSpeed = max( -absLimit, min( leftMotorSpeed, absLimit ) )
        self.rightMotorSpeed = max( -absLimit, min( rightMotorSpeed, absLimit ) )
    
    #-----------------------------------------------------------------------------------------------
    def setNeckAngles( self, panAngle, tiltAngle ):       
    
        self.panAngle = panAngle
        self.tiltAngle = tiltAngle
    
    #-----------------------------------------------------------------------------------------------
    def getMaxAbsMotorSpeed( self ):
        
        return self.MAX_ABS_MOTOR_SPEED
    
#---------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    # Create a Pyro server and register our module with it
    Pyro.core.initServer()
    ns = Pyro.naming.NameServerLocator().getNS()
    daemon = Pyro.core.Daemon()
    daemon.useNameServer( ns )
    uri = daemon.connect( RobotControl(), "robot_control_server" )
    daemon.requestLoop()
