import rospy
from vs_project.msg import MarkerPose
from vs_project.msg import detectedMarker
import numpy as np
from geometry_msgs.msg import Twist
import math
from tf.transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
import utils
import create_map
import cv2

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

def controller(homogenous_robot,homogenous_target):
    global robot_vel
    global parking_done
    global inital_orientation
    global vel_x
    global vel_z 
   
    homogenous_robot_inv = np.linalg.inv(homogenous_robot)
    Tcurgoal = np.dot(homogenous_robot_inv, homogenous_target)

    deltax =  Tcurgoal[0][3]
    deltay =  Tcurgoal[1][3]
    
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
    else:
        if  p < 0.045:
            robot_vel.linear.x = 0
            robot_vel.angular.z = 0
            pub.publish(robot_vel)
            rospy.sleep(0.01)
            print("ROBOT REACHED THE TARGET")
            inital_orientation = False
            return True
        else:
            if np.fabs(math.degrees(alfa)) > 15:
                if vel_x == 0:
                    v = 0
                    k_alpha = 0.8
                    w = k_alpha * alfa
                else:
                    v = 0      
                    robot_vel.linear.x = v
                    pub.publish(robot_vel)  
                    rospy.sleep(0.01)
                    return
            else:    
                w = 0  
                
                k_p = 0.8
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
    return False

def parking(homogenous_robot,homogenous_target):

    homogenous_robot_inv = np.linalg.inv(homogenous_robot)
    Tcurgoal = np.dot(homogenous_robot_inv, homogenous_target)

    rotation_matrix = [[Tcurgoal[0][0], Tcurgoal[0][1], Tcurgoal[0][2]], [Tcurgoal[1][0], Tcurgoal[1][1], Tcurgoal[1][2]], [Tcurgoal[2][0], Tcurgoal[2][1], Tcurgoal[2][2]]]    
    r = R.from_matrix(rotation_matrix)
    euler_degrees = r.as_euler('xyz', degrees=True)
       
    teta_error = euler_degrees[2]

    print("THETA ROBOT GOAL",teta_error)

    if np.fabs(teta_error) < 5:
        robot_vel.angular.z = 0  
        pub.publish(robot_vel)   
        print("PARKING DONE")
        return True 
    else:
        k_teta = 0.8
        w = k_teta * math.radians(euler_degrees[2])  

        if w > 2.84:
            w = 2.84
        elif w < -2.84:
            w = -2.84 

        robot_vel.linear.x = 0
        robot_vel.angular.z = w  
        pub.publish(robot_vel)   
        rospy.sleep(0.01) 
        return False

count = 0

def callback(msg):   
    global target_assigned
    global robot_assigned   
    global count
    global target_reached


    if msg.id == ROBOT_MARKER_ID:
        Rmat_robot = utils.rot_matrix(msg.rvec)
        homogenous_robot = utils.homogenous_matrix(Rmat_robot, msg.tvec)   
        robot_assigned = True
    elif msg.id == TARGET_MARKER_ID:  
        pass     
    else:
        raise ValueError("check your marker IDs")    

    if robot_assigned:
        if not target_reached:
            print("to target", count)
            print("total number of targets", len(target_matrices))
            result = controller(homogenous_robot,target_matrices[count])
            if result:
                count += 1
            if count == len(target_matrices):
                print("Robot reached the target, parking starts")
                target_reached = True
        elif not parking_done:
            result = parking(homogenous_robot,target_matrices[count-1])
            if result:
                print("Parking is done, mission completeeeed")
                rospy.signal_shutdown("DONE")
                cv2.destroyAllWindows() 
        robot_assigned = False 


def callback2(msg):
    global vel_x
    global vel_z 

    vel_x = msg.linear.x
    vel_z = msg.angular.z

utils.init()
rospy.init_node('controller5')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

target_matrices = create_map.map()

sub = rospy.Subscriber('/estimated_pose', MarkerPose, callback)
sub2 = rospy.Subscriber('/cmd_vel', Twist, callback2)
rospy.spin()