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

import os
import shutil
import filecmp
import subprocess

#---------------------------------------------------------------------------------------------------
class BoardInfo:
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self, name, processor, uploadSpeed ):
        
        self.name = name
        self.processor = processor
        self.uploadSpeed = uploadSpeed

#---------------------------------------------------------------------------------------------------
DEFAULT_SERIAL_PORT_NAME = "/dev/ttyUSB0"
DEFAULT_BOARD_MODEL = "uno"

BOARD_INFO_DICT = {
    "uno" : BoardInfo( "uno", "atmega328p", 115200 ),
    "atmega8" : BoardInfo( "atmega8", "atmega8", 19200 )
}

#---------------------------------------------------------------------------------------------------
def getInoUploaderUserDir():
    
    homeDir = os.environ[ "HOME" ]
    return homeDir + "/.ino_uploader"

#---------------------------------------------------------------------------------------------------
def upload( sketchDir, serialPortName=DEFAULT_SERIAL_PORT_NAME, 
    boardModel=DEFAULT_BOARD_MODEL, forceRebuild=False ):

    boardInfo = BOARD_INFO_DICT[ boardModel ]
    
    uploadSucceeded = False
    
    inoUploaderUserDir = getInoUploaderUserDir()
    
    sketchDirBasename = os.path.basename( sketchDir )
    if len( sketchDirBasename ) == 0:
        raise Exception( "Invalid sketch directory - " + sketchDir )
    
    inoUploaderSketchDir = inoUploaderUserDir + "/" + sketchDirBasename
    inoUploaderSrcDir = inoUploaderSketchDir + "/src"
    
    sketchFiles = os.listdir( sketchDir )
    
    # Check to see if we need to copy files over
    fileCopyNeeded = False
    if forceRebuild:
        
        if os.path.exists( inoUploaderSketchDir ):
            shutil.rmtree( inoUploaderSketchDir )
            
        fileCopyNeeded = True
        
    else:
        
        if not os.path.exists( inoUploaderSrcDir ):
            
            fileCopyNeeded = True
            
        else:
            
            match, mismatch, errors = filecmp.cmpfiles( sketchDir, inoUploaderSrcDir, sketchFiles )
            if len( mismatch ) > 0 or len( errors ) > 0:
                
                fileCopyNeeded = True
                
    # Copy files over if needed    
    if fileCopyNeeded:
        
        if os.path.exists( inoUploaderSrcDir ):
            shutil.rmtree( inoUploaderSrcDir )
            
        shutil.copytree( sketchDir, inoUploaderSrcDir )
        
    # Now try to build the sketch
    buildResult = subprocess.call( 
        [ "/usr/local/bin/ino", "build", "-m", boardModel ], cwd=inoUploaderSketchDir )
    
    # Upload if the build was successful
    if buildResult == 0:
        
        hexFilename = inoUploaderSketchDir + "/.build/{0}/firmware.hex".format( boardModel )
        
        print "Trying to upload", boardModel
        
        uploadResult = subprocess.call( [ "avrdude", 
            "-p", boardInfo.processor,
            "-P", serialPortName, "-c", "arduino", "-b", str( boardInfo.uploadSpeed ), 
            "-D", "-U", "flash:w:{0}:i".format( hexFilename ) ] )

        print "uploadResult", uploadResult
            
        if uploadResult == 0:
            uploadSucceeded = True
    
    return uploadSucceeded