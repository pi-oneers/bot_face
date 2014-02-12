This is the code for a Raspberry Pi based camera robot. The robot's motors are 
driven by an Arduino compatible Dagu Mini Driver board. Tornado is used to
provide a web interface for the robot which allows the robot to be driven
around whilst streaming images back from the Raspberry Pi's camera.

Installing
----------

This code has the following dependencies

* Your Raspberry Pi must have its camera enabled
* mjpg-streamer - a good tutorial is [here](http://blog.miguelgrinberg.com/post/how-to-build-and-run-mjpg-streamer-on-the-raspberry-pi)
* [PySerial](http://pyserial.sourceforge.net), [Tornado](http://www.tornadoweb.org), and [Ino](http://inotool.org/)

install with

    sudo apt-get update
    sudo apt-get install python-pip python-dev python-serial
    sudo pip install tornado ino
    
* [sockjs-tornado](https://github.com/mrjoes/sockjs-tornado) 

install with

    git clone https://github.com/mrjoes/sockjs-tornado.git
    cd sockjs-tornado
    sudo python setup.py install

Finally, clone this repository by running

    git clone https://bitbucket.org/DawnRobotics/raspberry_pi_camera_bot.git   
 
Running
-------

Start the server by running

    cd raspberry_pi_camera_bot
    sudo ./robot_web_server.py
    
Alternatively, get the server to start when the Raspberry Pi boots by running

    sudo cp init.d/robot_web_server /etc/init.d/robot_web_server
    sudo chmod a+x /etc/init.d/robot_web_server
    sudo update-rc.d robot_web_server defaults

