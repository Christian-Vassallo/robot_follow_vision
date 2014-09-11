This repository introduces a new way to control the robot to go through a door.
The wheeled robot Robulab is considered.
MoCap system observes the scenario and record the data.
Wireless Vivotek Camera 7137 is equipped on the top of the robot.
MATLAB files allows to plot the position of the robot in the room, its orientation, linear and angular velocity.

This work has been tested on ROS Groovy, Ubuntu 12.04.4 LTS 64bit.


Instruction

OpenCV v2.4.9 and VISP required.
VISP: http://www.irisa.fr/lagadic/visp/download.html
OpenCV: http://opencv.org/downloads.html

ROS Matlab Support: http://www.mathworks.fr/hardware-support/robot-operating-system.html?refresh=true

evart_bridge required to communicate with Cortex system (Motion Capture Analysis)
https://github.com/laas/motion_analysis_mocap

In particular here:
git clone --recursive git://github.com/laas/motion_analysis_mocap.git


CameraClass contains method to connect to the camera and extract data from it as video streaming live

mocap-tracking-beta allows you to test if MoCap communication works
ellipsedetection allows you to test if the ellipse are detected
testopencv allows you to test if opencv works (just circles detection)

features-detection analyses the videostreaming and gives information about position of features in the image plane (Xo, Yo are in top-left corner)
elliptical-coord-control is the main file: robot control, data processing and output for matlab to plot data.

MATLAB files
ROS_connection.m -> set ROS on matlab. remember to set the master URI correctly
main.m -> main program: receives data and plot them
xxxMSG.m -> files used for the subscribers


