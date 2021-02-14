# has-scope
HAS-Scope-Code

## Overview
This repository contains two code for two programs. The first is the Arduino code that runs on the Arduino microcontroller that is connected to the stepper motor drivers of the telescope. The other program is an INDI Server Driver that mediates between Stellarium and the Arduino. 

### nijuyon-scope-sim
The Arduino code uses the AccelStepper library to control the stepper motor drivers. 
https://www.arduino.cc/reference/en/libraries/accelstepper/

### has-scope-INDI-driver
This is a INDI driver, written in C++, which is for the INDI server that connects Stellarium to the Arduino. 
The driver is an child of the INDO telescope device class. 
https://indilib.org/

## Setup
The software stack of the INDI driver and Stellarium has been tested on Ubuntu and Raspbian. It was easier to setup on Ubuntu. Charlie compiled the latest version of Stellarium specifically for the Raspbery Pi.  
