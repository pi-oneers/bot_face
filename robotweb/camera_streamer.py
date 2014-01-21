import copy
import os
import os.path
import subprocess
import time

#---------------------------------------------------------------------------------------------------
class CameraStreamer:
    
    """A class to look after streaming images from the Raspberry Pi camera.
       Ideally, the camera should only be on when somebody wants to stream images.
       Therefore, startStreaming must be called periodically. If startStreaming
       is not called before a timeout period expires then the streaming will stop"""

    IMAGE_PATH = "/tmp/stream"
    IMAGE_NAME = "pic.jpg"
    DEFAULT_TIMEOUT = 4.0
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, timeout=DEFAULT_TIMEOUT ):
            
        self.cameraProcess = None
        self.mjpgStreamerProcess = None
        self.streamingStartTime = 0
        self.streamingTimeout = timeout

    #-----------------------------------------------------------------------------------------------
    def __del__( self ):

        self.stopStreaming()
        
    #-----------------------------------------------------------------------------------------------
    def startStreaming( self ):
        
        fullImagePath = self.IMAGE_PATH + "/" + self.IMAGE_NAME
        
        # Create streaming path if needed
        if not os.path.exists( self.IMAGE_PATH ):
            
            os.makedirs( self.IMAGE_PATH )
        
        # Start raspistill if needed
        if self.cameraProcess == None or self.cameraProcess.poll() != None:
            
            self.cameraProcess = subprocess.Popen( [ "raspistill",
                "--nopreview", "-w", "640", "-h", "480", "-q", "5",
                "-o", fullImagePath, "-tl", "100", "-t", "9999999", "-th", "0:0:0" ] )
        
        # Start mjpg_streamer if needed
        if self.mjpgStreamerProcess == None or self.mjpgStreamerProcess.poll() != None:
            
            self.mjpgStreamerProcess = subprocess.Popen( [ 
                "/usr/local/bin/mjpg_streamer",
                "-i", "/usr/local/lib/input_file.so -f {0} -n {1}".format( self.IMAGE_PATH, self.IMAGE_NAME ), 
                "-o", "/usr/local/lib/output_http.so -w /usr/local/www" ] )

        self.streamingStartTime = time.time()         
                
    #-----------------------------------------------------------------------------------------------
    def update( self ):
        
        if time.time() - self.streamingStartTime > self.streamingTimeout:
            
            self.stopStreaming()
            
        #else:
            #if self.cameraProcess != None:
                #print "Camera", self.cameraProcess.poll()
            
            #if self.mjpgStreamerProcess != None:
                #print "mjpg", self.mjpgStreamerProcess.poll()
                
    #-----------------------------------------------------------------------------------------------
    def stopStreaming( self ):

        if self.cameraProcess != None:
            self.cameraProcess.terminate()
            
        if self.mjpgStreamerProcess != None:
            self.mjpgStreamerProcess.terminate()