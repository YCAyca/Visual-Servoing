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

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

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
  # image = PImage.frombytes("RGB", (msg.height,msg.width), msg.data)

  # image = np.array(image.getdata(), dtype=np.uint8).reshape(msg.height, msg.width, -1)

  image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
  
  newcameramtx, roi = cv2.getOptimalNewCameraMatrix(calib_m, dist_coefs, (3,3), 1, (3,3))
  

  image = cv2.undistort(image, calib_m, dist_coefs, newCameraMatrix=newcameramtx)
  # cv2.imshow("undistorted image", image)
  # cv2.waitKey(0)

  (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict,parameters=arucoParams)

  print(type(corners))
  print(corners)

  if len(corners) > 0:
    # loop over the detected ArUCo corners
    for (markerCorner, markerID) in zip(corners, ids):
      # Draw a square around the markers
      # cv2.aruco.drawDetectedMarkers(image, corners) 
      #estimate pose
      rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.02, calib_m, dist_coefs)  

      print("ROTATION VECTOR",rvec)
      print("TRANLATION VECTOR", tvec)
      # Draw Axis
      cv2.aruco.drawAxis(image, calib_m, dist_coefs, rvec, tvec, 0.01)  

     

      # publish the estimated pose with tranlation and rotation vectors

      #pub.publish(rvec)


  cv2.imshow("pose estimated image", image)
  cv2.waitKey(1)

 

  # publish the image to detected_markers topic
  
  # image_message = bridge.cv2_to_imgmsg(image)
  # pub.publish(image_message)



arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)  # for simulation
arucoParams = cv2.aruco.DetectorParameters_create() 
bridge = CvBridge()


proj_m, calib_m, dist_coefs = read_calibration_file("/home/yca/catkin_ws/src/vs_project/ost.yaml")

rospy.init_node('pose_estimation')
pub = rospy.Publisher('/estimated_pose', UInt8MultiArray, queue_size=1)


while not rospy.is_shutdown(): 
  sub = rospy.Subscriber('/t265/stereo_ir/left/fisheye_image_raw', Image, callback)
  rospy.spin()

cv2.destroyAllWindows()  