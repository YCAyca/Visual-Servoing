#! /usr/bin/env python

import rospy
import PIL.Image as PImage
import io
import base64
from sensor_msgs.msg import Image 
from std_msgs.msg import Int16
import cv2
from cv_bridge import CvBridge
import numpy as np
import yaml
from std_msgs.msg import UInt8MultiArray

def read_calibration_file(file_name):
  with open(file_name, "r") as file:
    try:
        documents = yaml.full_load(file)
    except yaml.YAMLError as exc:
        print(exc)

    distortion_coefs = np.array(documents.get('distortion_coefficients').get('data'))
    camera_matrix = documents.get('camera_matrix').get('data')
    projection_matrix = documents.get('projection_matrix').get('data')

    camera_matrix = np.array(camera_matrix)
    camera_matrix = np.reshape(camera_matrix, (3,3))
    
    projection_matrix = np.array(projection_matrix)
    projection_matrix = np.reshape(projection_matrix, (3,4))

    print("D",distortion_coefs)
    print("C",camera_matrix)
    print("P",projection_matrix)

    return projection_matrix,  camera_matrix, distortion_coefs



def callback(msg):    
  image = PImage.frombytes("RGB", (msg.height,msg.width), msg.data)

  image = np.array(image.getdata(), dtype=np.uint8).reshape(msg.height, msg.width, -1)

  cv2.imshow("pure image", image)
  cv2.waitKey(0)

  image = cv2.undistort(image, calib_m, dist_coefs, newCameraMatrix=calib_m)
  cv2.imshow("undistorted image", image)
  cv2.waitKey(0)

  (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,parameters=arucoParams)

  print(type(corners))
  print(corners)

  if len(corners) > 0:
    # flatten the ArUco IDs list
    ids = ids.flatten()
    # loop over the detected ArUCo corners
    for (markerCorner, markerID) in zip(corners, ids):
      # Draw a square around the markers
      cv2.aruco.drawDetectedMarkers(image, corners) 

      cv2.imshow("marker detected image", image)
      cv2.waitKey(0)
 
      #estimate pose
      rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, 0.02, calib_m, dist_coefs)  

      # Draw Axis
      cv2.aruco.drawAxis(image, calib_m, dist_coefs, rvec, tvec, 0.01)  

      cv2.imshow("pose estimated image", image)
      cv2.waitKey(0)

      print("ROTATION VECTOR",rvec)
      print("TRANLATION VECTOR", tvec)


  cv2.destroyAllWindows()   

  # publish the image to detected_markers topic
  
  # image_message = bridge.cv2_to_imgmsg(image)
  # pub.publish(image_message)

  # publish the estimated pose with tranlation and rotation vectors

 




rospy.init_node('marker_detector')
pub = rospy.Publisher('/detected_markers', Image, queue_size=1)

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
arucoParams = cv2.aruco.DetectorParameters_create() 
bridge = CvBridge()

proj_m, calib_m, dist_coefs = read_calibration_file("ost.yaml")

while not rospy.is_shutdown(): 
   sub = rospy.Subscriber('/t265/stereo_ir/left/fisheye_image_raw', Image, callback)
   rospy.spin()



