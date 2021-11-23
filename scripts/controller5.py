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
TARGET_MARKER_ID = 1  
target_assigned = False
robot_assigned = False
target_reached = False
parking_done = False
inital_orientation = False

robot_pose = []

robot_vel = Twist()

robot_vel.linear.x = 0
robot_vel.linear.y = 0
robot_vel.linear.z = 0

robot_vel.angular.x = 0
robot_vel.angular.y = 0
robot_vel.angular.z = 0

vel_x = 0
vel_z = 0


def rot_matrix(rvec):
    Rmat, _ = cv2.Rodrigues(rvec)
    return Rmat

def homogenous_matrix(rotmat, tvec):
    return [[rotmat[0][0], rotmat[0][1], rotmat[0][2], tvec[0]], [rotmat[1][0], rotmat[1][1], rotmat[1][2], tvec[1]],  [rotmat[2][0], rotmat[2][1], rotmat[2][2], tvec[2]], [0, 0, 0, 1]]

def controller(homogenous_robot,homogenous_target):
    global robot_vel
    global parking_done
    global target_reached
    global inital_orientation
    global vel_x
    global vel_z 
     
  #  print("\nHOMO ROBO \n",homogenous_robot)
  #  print("\nHOMO TARGET \n" , homogenous_target)
   
    homogenous_robot_inv = np.linalg.inv(homogenous_robot)

    Tcurgoal = np.dot(homogenous_robot_inv, homogenous_target)

    print("\nT CUR GOAL\n", Tcurgoal)

    deltax =  Tcurgoal[0][3]
    deltay =  Tcurgoal[1][3]

  #  print("deltax", deltax)
  #  print("deltay", deltay)
    
    alfa = np.arctan2(deltay, deltax)
    
    print("ALFA", np.degrees(alfa))

    p = math.sqrt(deltax**2 + deltay**2) 

    print("DISTANCE", p)

    if not inital_orientation:
        if np.fabs(math.degrees(alfa)) > 5:
            k_alpha = 1.2
            v = 0
            w = k_alpha * alfa
        else:
            inital_orientation = True
            print("INITIAL ORIENTATION DONE")
            w = 0
            v = 0

        if w > 2.84:
            w = 2.84
        elif w < -2.84:
            w = -2.84 

        robot_vel.angular.z = w  
        robot_vel.linear.x = v  

        pub.publish(robot_vel)  
        rospy.sleep(0.01)
    

    elif not target_reached:
        if  p < 0.045:
            robot_vel.linear.x = 0
            robot_vel.angular.z = 0
            target_reached = True
            pub.publish(robot_vel)
            print("ROBOT REACHED THE TARGET")
            rospy.sleep(0.01)
        #    rospy.signal_shutdown("ROBOT REACHED THE TARGET")
            

        if np.fabs(math.degrees(alfa)) > 15:
            if vel_x == 0:
                v = 0
                k_alpha = 0.5
                w = k_alpha * alfa
            else:
               v = 0      
               robot_vel.linear.x = v
               pub.publish(robot_vel)  
               rospy.sleep(0.01)
               return
        else:
            w = 0  
            k_p = 0.25
            v = k_p * p

    
        if v > 0.22:
            v = 0.22
        elif v < -0.22:
            v = -0.22   

        if w > 2.84:
            w = 2.84
        elif w < -2.84:
            w = -2.84 

        robot_vel.angular.z = w  
        robot_vel.linear.x = v  

        pub.publish(robot_vel)  
        rospy.sleep(0.01)

    elif not parking_done:

        rotation_matrix = [[Tcurgoal[0][0], Tcurgoal[0][1], Tcurgoal[0][2]], [Tcurgoal[1][0], Tcurgoal[1][1], Tcurgoal[1][2]], [Tcurgoal[2][0], Tcurgoal[2][1], Tcurgoal[2][2]]]    
        r = R.from_matrix(rotation_matrix)
        euler_degrees = r.as_euler('xyz', degrees=True)
       
        teta_error = normalize(euler_degrees[2])

        print("THETA ROBOT GOAL", math.degrees(teta_error))

        if np.fabs(math.degrees(teta_error)) < 5:
            parking_done = True
            robot_vel.angular.z = 0  
            pub.publish(robot_vel)   
            print("PARKING DONE")               
            rospy.signal_shutdown("PARKING DONE")
        else:
            k_teta = 0.08
            w = k_teta * euler_degrees[2]  

            if w > 2.84:
                w = 2.84
            elif w < -2.84:
                w = -2.84 

            robot_vel.linear.x = 0
            robot_vel.angular.z = w  
            pub.publish(robot_vel)   
            rospy.sleep(0.01) 


def normalize(angle):
    return np.arctan2(np.sin(angle),np.cos(angle))

def callback(msg):   
    global target_assigned
    global robot_assigned   
    global homogenous_robot
    global homogenous_target

    if msg.id == ROBOT_MARKER_ID:
        Rmat_robot = rot_matrix(msg.rvec)
        homogenous_robot = homogenous_matrix(Rmat_robot, msg.tvec)   
        robot_assigned = True
    elif msg.id == TARGET_MARKER_ID:  # no need to assign target possition again and again
        if target_assigned == False:
          target_assigned = True
          Rmat_target = rot_matrix(msg.rvec)
          homogenous_target = homogenous_matrix(Rmat_target, msg.tvec)        
    else:
        raise ValueError("check your marker IDs")    

    if target_assigned and robot_assigned:
        controller(homogenous_robot,homogenous_target)
        robot_assigned = False 

def callback2(msg):
    global vel_x
    global vel_z 

    vel_x = msg.linear.x
    vel_z = msg.angular.z



rospy.init_node('controller5')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

sub = rospy.Subscriber('/estimated_pose', MarkerPose, callback)
sub2 = rospy.Subscriber('/cmd_vel', Twist, callback2)
rospy.spin()