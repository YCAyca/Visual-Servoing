import yaml
import cv2
import numpy as np


def init():
	global target_matrices
	global ENV_IMAGE_NAME
	global MARKER_IMAGE_NAME

	ENV_IMAGE_NAME = "/home/yca/catkin_ws/src/vs_project/obs4.png"
	MARKER_IMAGE_NAME ="/home/yca/catkin_ws/src/vs_project/marker.png"

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


def rot_matrix(rvec):
    Rmat, _ = cv2.Rodrigues(rvec)
    return Rmat

def homogenous_matrix(rotmat, tvec):
    return [[rotmat[0][0], rotmat[0][1], rotmat[0][2], tvec[0]], [rotmat[1][0], rotmat[1][1], rotmat[1][2], tvec[1]],  [rotmat[2][0], rotmat[2][1], rotmat[2][2], tvec[2]], [0, 0, 0, 1]]
