import rospy
from vs_project.msg import MarkerPose
from vs_project.msg import detectedMarker
import numpy as np
from geometry_msgs.msg import Twist
import sys
import math

ROBOT_MARKER_ID = 0
TARGET_MARKER_ID = 3  
target_assigned = False
robot_assigned = False
target_reached = False


robot_pose = []
target_pose = []

forward_speed_gain = 0.7
rotational_speed_gain = 0.5

robot_vel = Twist()

robot_vel.linear.x = 0
robot_vel.linear.y = 0
robot_vel.linear.z = 0

robot_vel.angular.x = 0
robot_vel.angular.y = 0
robot_vel.angular.z = 0
        

def controller(robot, target):
    global target_reached
    global robot_vel

    print("robot pose",robot[0][1][1])
    print("robot pose",robot)

    distance = distance_to_target(target,robot)
    
    if  distance < 0.01:
        robot_vel.linear.x = 0
        robot_vel.angular.z = 0
        
        pub.publish(robot_vel)
        print("robot reached th target ciaou")
        rospy.signal_shutdown("robot reached th target ciaou")

    teta_error = math.degrees(target[0][0][2] - robot[0][0][2]) # z angle

    print("teta error",teta_error)

    teta_error2 = math.degrees(math.atan((target[0][1][1] - robot[0][1][1]) / (target[0][1][0] - robot[0][1][0])))  # target_y - robot_y / target_x - robot_x

    print("teta error2",teta_error2)

    if math.fabs(teta_error2) > 10:
        robot_vel.linear.x = 0
       
        speed_w = compute_rotational_speed(distance)
        if teta_error2 > 0:
            speed_w *= -1
        robot_vel.angular.z = speed_w
    else:
        speed_f = compute_forward_speed(distance)
        speed_w = compute_rotational_speed(distance)
        robot_vel.linear.x =  speed_f
        robot_vel.angular.z = speed_w
            
    pub.publish(robot_vel)    

def callback(msg):   
    global target_assigned
    global robot_assigned   

    if msg.id == ROBOT_MARKER_ID:
     #   print("robot uv") 
        robot_pose.clear()
        robot_pose.append((msg.rvec, msg.tvec))
        robot_assigned = True
    elif msg.id == TARGET_MARKER_ID:  # no need to assign target possition again and again
        #   print("target uv") 
     #   if not target_assigned:
        target_pose.clear()
        target_pose.append((msg.rvec, msg.tvec))
        target_assigned = True
    else:
        raise ValueError("check your marker IDs")    

    if target_assigned and robot_assigned:
        controller(robot_pose, target_pose)
        robot_assigned = False # we should be sure if the new robot position came before calling controller serially
        target_assigned = False # no need if the target doesnt move
        

def distance_to_target(target, robot):
        return math.sqrt((target[0][1][0]-robot[0][1][0])**2 + (target[0][1][1]-robot[0][1][1])**2)   # (target_x-pos_x)**2 + (target_y-pos_y)**2)        


def compute_rotational_speed(distance):
        return rotational_speed_gain*distance

def compute_forward_speed(distance):
        return forward_speed_gain*distance

rospy.init_node('controller2')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10)


sub = rospy.Subscriber('/estimated_pose', MarkerPose, callback)
rospy.spin()

  