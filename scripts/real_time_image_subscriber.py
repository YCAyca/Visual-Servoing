#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image 
from std_msgs.msg import Int16
import cv2
from cv_bridge import CvBridge
import numpy as np
import yaml
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import UInt8
from vs_project.msg import MarkerPose
from vs_project.msg import detectedMarker
import utils

utils.init()

env_initialized = False

def callback(msg):   
  global env_initialized 
  image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
  
  newcameramtx, roi = cv2.getOptimalNewCameraMatrix(calib_m, dist_coefs, (3,3), 1, (3,3))
  
  image = cv2.undistort(image, calib_m, dist_coefs, newCameraMatrix=newcameramtx)

  if not env_initialized:
    cv2.imwrite(utils.ENV_IMAGE_NAME, cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    env_initialized = True

  (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,parameters=arucoParams)

  if len(corners) > 0:
    # loop over the detected ArUCo corners
    for (markerCorner, markerID) in zip(corners, ids):

      rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.15, calib_m, dist_coefs)  
      
      # Draw Axis
      cv2.aruco.drawAxis(image, calib_m, dist_coefs, rvec, tvec, 0.1)  

      #publish estimated pose 
    
      pose = MarkerPose()
      pose.id = markerID[0]
      pose.rvec = rvec[0][0]
      pose.tvec = tvec[0][0]
      pub.publish(pose)
      
          
  cv2.imshow("pose estimated image", image)
  cv2.waitKey(1)


######

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
arucoParams = cv2.aruco.DetectorParameters_create() 
bridge = CvBridge()


proj_m, calib_m, dist_coefs = utils.read_calibration_file("ost_real.yaml")

rospy.init_node('pose_estimation')
pub = rospy.Publisher('/estimated_pose', MarkerPose, queue_size=1)

while not rospy.is_shutdown(): 
  sub = rospy.Subscriber('/camera/image_raw', Image, callback)
  rospy.spin()

cv2.destroyAllWindows()  





