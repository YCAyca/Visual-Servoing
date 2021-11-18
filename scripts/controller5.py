import rospy
from vs_project.msg import MarkerPose
from vs_project.msg import detectedMarker
import numpy as np
from geometry_msgs.msg import Twist
import sys
import math
from tf.transformations import euler_from_quaternion
import cv2
from scipy.spatial.transform import Rotation as R
import yaml
import time

ROBOT_MARKER_ID = 0
TARGET_MARKER_ID = 1  
target_assigned = True
robot_assigned = True

forward_speed_gain = 1
rotational_speed_gain = 1

robot_vel = Twist()

robot_vel.linear.x = 0
robot_vel.linear.y = 0
robot_vel.linear.z = 0

robot_vel.angular.x = 0
robot_vel.angular.y = 0
robot_vel.angular.z = 0

k_p = 0.3
k_alpha = 0.8
k_beta = -0.15

v = 0
w = 0


def get_target_pos(file_name):
  with open(file_name, "r") as file:
    try:
        documents = yaml.full_load(file)
    except yaml.YAMLError as exc:
        print(exc)

    target_matrix = np.array(documents.get('homogenous_matrix').get('data'))    
    target_teta = documents.get('teta_target').get('data')  

    return target_matrix, target_teta


def rot_matrix(rvec):
    Rmat, _ = cv2.Rodrigues(rvec)
    return Rmat


def homogenous_matrix(rotmat, tvec):
    return [[rotmat[0][0], rotmat[0][1], rotmat[0][2], tvec[0]], [rotmat[1][0], rotmat[1][1], rotmat[1][2], tvec[1]],  [rotmat[2][0], rotmat[2][1], rotmat[2][2], tvec[2]], [0, 0, 0, 1]]


def controller(Tcurcam, robot_teta, Tgoalcam, target_teta):
    global robot_vel
    global v
    global w

    Tgoalcam_inv = np.linalg.inv(Tgoalcam)

    Tcurgoal =  Tcurcam * Tgoalcam_inv 

    Tcurgoal = np.dot(Tcurcam, Tgoalcam_inv)

    print("T CUR GOAL", Tcurgoal)

    deltax =  Tcurgoal[0][3]
    deltay =  Tcurgoal[1][3]

    print("deltax", deltax)
    print("deltay", deltay)
    
    rotation_matrix = [[Tcurgoal[0][0], Tcurgoal[0][1], Tcurgoal[0][2]], [Tcurgoal[1][0], Tcurgoal[1][1], Tcurgoal[1][2]], [Tcurgoal[2][0], Tcurgoal[2][1], Tcurgoal[2][2]]]    

    print("cur goal rotation matrix", rotation_matrix)

    r = R.from_matrix(rotation_matrix)

    rvec = r.as_rotvec()

    teta = rvec[2]

    print("TETA ROBOT GOAL", teta)

   # teta2 = np.arctan2(deltay, deltax)

    t_k = time.time()
    pos_x = Tcurcam[0][3]
    pos_y = Tcurcam[1][3]

    target_x = Tgoalcam[0][3]
    target_y = Tgoalcam[1][3]

    while True:
      p = math.sqrt(deltax**2 + deltay**2) 

      print("distance", p)
      if p < 0.2:
        break

      alfa = teta - robot_teta 

      if math.fabs(alfa) > 360:
        alfa %= 360
      
      print("robot to cam teta",teta)

      print("robot teta",robot_teta)

      print("ALFA (TETA ERROR)", alfa)

      if math.degrees(alfa) > 10:
        v = 0
      else:
        v = k_p * p

      w = k_alpha * alfa

      if alfa < 0:
        w *= -1

      if math.fabs(w) > 2.84:
        w = 0.2

      if math.fabs(v) > 0.22:
        v = 0.2  

      robot_vel.angular.z = w  
      robot_vel.linear.x = v
      pub.publish(robot_vel)   
      current_time = time.time()
      delta_t = current_time -  t_k
      robot_teta = robot_teta + robot_vel.angular.z * delta_t
      pos_x = pos_x + (robot_vel.linear.x * math.cos(robot_teta)) * delta_t
      pos_y = pos_y + (robot_vel.linear.x * math.sin(robot_teta)) * delta_t
      p = math.sqrt((target_x-pos_x)**2 + (target_y-pos_y)**2)
      teta = math.degrees(math.atan((target_y - pos_y) / (target_x - pos_x))) - robot_teta
      t_k = time.time()



  
    robot_vel.linear.x = 0
    robot_vel.angular.z = 0
    
    pub.publish(robot_vel)
    print("robot reached th target ciaou")
    rospy.signal_shutdown("robot reached th target ciaou")


def callback(msg):   
    Rmat_robot = rot_matrix([3.16506197, -0.07426953, -0.06763119])
    homogenous_robot = homogenous_matrix(Rmat_robot, [-0.69233909,  0.0326587,   1.06192585])

    Rmat_target =  rot_matrix([0.01036359, 3.13365486, 0.05132683])       
    homogenous_target = homogenous_matrix(Rmat_target, [-0.44558346, -0.82360428,  1.04953636])
        
       
 #   controller(homogenous_robot, robot_teta, homogenous_target, target_teta)
  




rospy.init_node('controller5')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(1)

sub = rospy.Subscriber('/estimated_pose', MarkerPose, callback)
rospy.spin()