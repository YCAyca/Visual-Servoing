## Visual Servoing Project Report

## Contents

* Introduction 
    * Project Objective
    * Tools

* Tasks and Their Implementation 
    * Camera Calibration
    * Real Time Image Handling
    * Map Creation
    * Shortest Path Finding with A* Algorithm
    * Pose Estimation of Robot and Way Points
    * Robot Navigation to the Way Points
    * Parking

* Resources & Conclusion 


## Introduction

Path finding and navigation is a frequently encountered task in the field
of robotics. Although there are many different methods of transporting a
mobile robot from its starting position to a specific destination, in this
project, we tried to fulfill this task by using a hand to eye visual
servoing system.

### Project Objective

The goal of the project is to control a robot using a fish eye camera fixed on the top of the environment using hand to eye visual servo system. The main purpose is to navigate the robot via this camera to the given target point avoiding obstacles.

### Tools

The hardware and the software tools used for this project:  
<br>
• Turtlebot3 Burger  
• FE185C057HA-1 Fish eye Camera  
• Ubuntu 20.4  
• ROS Noetic


## Tasks and Their Implementation

We implemented this project full of different tasks and challenges, by dividing it into smaller steps. The flow chart below shows the general steps and operations of the project. <br>
Therefore, we regularly receive images from our fish-eye camera overlooks the environment, and use this information to calculate the initial position of the robot and the target, only once at the beginning. After transforming this positions into a maze map, we use the A* algorithm, we calculate the shortest path between the robot and target taking into account the obstacles. The way points we obtain as a result of the A* algorithm
become intermediate target points that the robot must reach one by one to reach the main target.
From this step on, we transform image frame (pixel-wise) positions to the camera frame positions and then calculate the appropriate speed to be sent to the robot.
After this transformation, the following cycle continues until the robot reaches the target point:
* Calculating the current position of the robot first in the camera coordinate system then in the
coordinate system of the way point to be reached. 
* Calculation of linear x and angular z speeds using
PID controller. 
* When the euclidian distance between the current position of the robot and the way point position is less than 0.05 m, the position of the next way point is loaded and the same process is applied until the last target point.

After reaching the final target point, parking as the last task is performed by orienting the robot with the same direction of
the target pose.

<img src="images/flow_chart.png" width=100% height=100%>

### Camera Calibration

To be able to use the images coming from the camera, we need to undistort them. For this purpose we calibrated the camera using a checkerboard and obtained the camera intrinsic, extrinsic parameters with distortion coefficients.

We used ROS camera_calibration package for this step with following command:  
<br>
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.08
image:=/camera/image_raw camera:=/camera   
The package information is given in the following link:  
<br>
http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

Our intrinsic camera calibration parameters obtained and used during the project is as follows:

<img src="images/calib.png" width=50% height=50%>

### Real Time Image Handling


Using the fish eye camera on the top of the environment, we obtain the first image to extract initial robot position and target position using the <b> Aruco Markers having 15x15 cm size </b>. For this purpose we have a subscriber listens to the <b> camera/image_raw </b> topic (the topic where our camera publishes the images). In the callback function of this subscriber, we detect the 4 corner of the markers as the pixel
coordinates in the image. This first image and initial pixel positions are used for map creation in the next step.