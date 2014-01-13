#!/usr/bin/env python

import time
import os
import Pyro.core
import Pyro.naming

#---------------------------------------------------------------------------------------------------
class Movement(Pyro.core.ObjBase):
    # Movement constants
    SPEED = 100

    # Set up the GPIO
    def _setup(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(24,GPIO.OUT)
        GPIO.setup(23,GPIO.OUT)
        GPIO.setup(25,GPIO.OUT)
        GPIO.setup(9,GPIO.OUT)
        GPIO.setup(10,GPIO.OUT)
        GPIO.setup(11,GPIO.OUT)

        self.Motor1 = GPIO.PWM(25, 50)
        self.Motor1.start(0)
        self.Motor2 = GPIO.PWM(11, 50)
        self.Motor2.start(0)

    def __init__(self):
        Pyro.core.ObjBase.__init__(self)
        self._setup()

    def stop(self):        
        self.Motor1.ChangeDutyCycle(0)
        self.Motor2.ChangeDutyCycle(0)

    def forward(self, move_time):
        GPIO.output(24,GPIO.HIGH)
        GPIO.output(23,GPIO.LOW)
        GPIO.output(9,GPIO.HIGH)
        GPIO.output(10,GPIO.LOW)
        self.Motor1.ChangeDutyCycle(Movement.SPEED)
        self.Motor2.ChangeDutyCycle(Movement.SPEED)
        
        sleep(move_time)
        self.stop()

    def backward(self, move_time):
        GPIO.output(24,GPIO.LOW)
        GPIO.output(23,GPIO.HIGH)
        GPIO.output(9,GPIO.LOW)
        GPIO.output(10,GPIO.HIGH)
        self.Motor1.ChangeDutyCycle(Movement.SPEED)
        self.Motor2.ChangeDutyCycle(Movement.SPEED)
        
        sleep(move_time)
        self.stop()

    def left(self, move_time):
        GPIO.output(24,GPIO.HIGH)
        GPIO.output(23,GPIO.LOW)
        self.Motor1.ChangeDutyCycle(Movement.SPEED)

        sleep(move_time)
        self.stop()
    
    def right(self, move_time):
        GPIO.output(9,GPIO.HIGH)
        GPIO.output(10,GPIO.LOW)
        self.Motor2.ChangeDutyCycle(Movement.SPEED)

        sleep(move_time)
        self.stop()

if __name__ == "__main__":
    # Create a Pyro server and register our module with it
    Pyro.core.initServer()
    ns = Pyro.naming.NameServerLocator().getNS()
    daemon = Pyro.core.Daemon()
    daemon.useNameServer(ns)
    uri = daemon.connect(Movement(),"robotmovement")
    daemon.requestLoop()
