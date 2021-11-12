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

ROBOT_MARKER_ID = 0
TARGET_MARKER_ID = 3  
target_assigned = False
robot_assigned = False

robot_pose = []
target_pose = []

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

def rot_matrix(rvec):
    Rmat, _ = cv2.Rodrigues(rvec)
    print(Rmat)
    return Rmat

def homogenous_matrix(rotmat, tvec):
    return [[rotmat[0][0], rotmat[0][1], rotmat[0][2], tvec[0]], [rotmat[1][0], rotmat[1][1], rotmat[1][2], tvec[1]],  [rotmat[2][0], rotmat[2][1], rotmat[2][2], tvec[2]], [0, 0, 0, 1]]

def controller(robot, target):
    global robot_vel

    Rmat_robot = rot_matrix(robot[0][0])
    Rmat_target = rot_matrix(target[0][0])

    print("ROTMAT",Rmat_robot)
    print("TVEC",robot[0][1])

    homogenous_robot = homogenous_matrix(Rmat_robot, robot[0][1])
    homogenous_target = homogenous_matrix(Rmat_target, target[0][1])
        
    print("HOMO ROBO",homogenous_robot)
    print("HOMO TARGET", homogenous_target)
    
    Tcurcam = homogenous_robot
    Tgoalcam = homogenous_target
    Tgoalcam_inv = np.linalg.inv(Tgoalcam)

    print( "T CURCAM",Tcurcam)
    print( "Tgoalcam",Tgoalcam)
    print( "Tgoalcam_inv",Tgoalcam_inv)

    Tcurgoal =  np.matmul(Tcurcam, Tgoalcam_inv)

    print("T CUR GOAL", Tcurgoal)

    deltax =  Tcurgoal[0][3]
    deltay =  Tcurgoal[1][3]

    print("deltax", deltax)
    print("deltay", deltay)
    
    rotation_matrix = [[Tcurgoal[0][0], Tcurgoal[0][1], Tcurgoal[0][2]], [Tcurgoal[1][0], Tcurgoal[1][1], Tcurgoal[1][2]], [Tcurgoal[2][0], Tcurgoal[2][1], Tcurgoal[2][2]]]    

    print("rotation matrix", rotation_matrix)

    r = R.from_matrix(rotation_matrix)

    rvec = r.as_rotvec()

    print("RVEC",rvec)

    teta = math.degrees(rvec[2])

    p = math.sqrt(deltax**2 + deltay**2) 

    print("distance", p)

    # joaquin's method
    
    # if  p < 0.03:
    #     robot_vel.linear.x = 0
    #     robot_vel.angular.z = 0
        
    #     pub.publish(robot_vel)
    #     print("robot reached th target ciaou")
    #     rospy.signal_shutdown("robot reached th target ciaou")

    # if math.fabs(teta) > 10:
    #     robot_vel.linear.x = 0
       
    #     speed_w = compute_rotational_speed(p)
    #     if teta < 0:
    #         speed_w *= -1
    #     robot_vel.angular.z = speed_w / 5
    # else:
    #     speed_f = compute_forward_speed(p)
    #     speed_w = compute_rotational_speed(math.radians(teta))

    #     if teta < 0:
    #         speed_w *= -1

    #     robot_vel.linear.x =  speed_f 
    #     robot_vel.angular.z = speed_w 
                 
   
    
    if  p < 0.05:
        robot_vel.linear.x = 0
        robot_vel.angular.z = 0
        
        pub.publish(robot_vel)
        print("robot reached th target ciaou")
        rospy.signal_shutdown("robot reached th target ciaou")


    alfa = math.degrees(math.atan(deltay/deltax)) - teta
    beta = -alfa - teta 

    v = k_p * p
    w = k_alpha * alfa + k_beta * beta

    robot_vel.linear.x =  v 
    robot_vel.angular.z = w / 100
            
    pub.publish(robot_vel)    



def compute_rotational_speed(distance):
        return rotational_speed_gain*distance

def compute_forward_speed(distance):
        return forward_speed_gain*distance

def callback(msg):   
    global target_assigned
    global robot_assigned   

    if msg.id == ROBOT_MARKER_ID:
        robot_pose.clear()
        robot_pose.append((msg.rvec, msg.tvec))
        robot_assigned = True
    elif msg.id == TARGET_MARKER_ID:  # no need to assign target possition again and again
        target_pose.clear()
        target_pose.append((msg.rvec, msg.tvec))
        target_assigned = True
    else:
        raise ValueError("check your marker IDs")    

    if target_assigned and robot_assigned:
        controller(robot_pose, target_pose)
        robot_assigned = False 




rospy.init_node('controller5')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(1)


sub = rospy.Subscriber('/estimated_pose', MarkerPose, callback)
rospy.spin()