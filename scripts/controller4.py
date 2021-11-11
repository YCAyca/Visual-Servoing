import rospy
from vs_project.msg import MarkerPose
from vs_project.msg import detectedMarker
import numpy as np
from geometry_msgs.msg import Twist
import sys
import math
from tf.transformations import euler_from_quaternion


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

def create_matrix(alfa, beta, p): 
    return [[-math.cos(alfa), 0], [math.sin(alfa)/p, -1], [-math.sin(alfa)/p, 0]]


def controller(robot, target):
    global robot_vel

    teta = robot[0][0][2]

  #  (_, _, teta) = euler_from_quaternion([robot[0][0][0], robot[0][0][1], robot[0][0][2], 0])  

    teta = math.degrees(teta)
    print(teta)
                  

    p = math.sqrt((target[0][1][0]-robot[0][1][0])**2 + (target[0][1][1]-robot[0][1][1])**2) 

    print("distance", p)
    
    if  p < 0.05:
        robot_vel.linear.x = 0
        robot_vel.angular.z = 0
        
        pub.publish(robot_vel)
        print("robot reached th target ciaou")
        rospy.signal_shutdown("robot reached th target ciaou")


    alfa = math.degrees(math.atan((target[0][1][1] - robot[0][1][1]) / (target[0][1][0] - robot[0][1][0]))) - teta
    beta = -alfa - teta 

    v = k_p * p
    w = k_alpha * alfa + k_beta * beta

    robot_vel.linear.x =  v 
    robot_vel.angular.z = w / 100
            
    pub.publish(robot_vel)    

    # matrix1 = np.array(create_matrix(alfa, beta, p))
    # matrix2 = np.array([[v], [w]])

    # print("m1",matrix1)
    # print("m2",matrix2)

    # np.matmul(matrix1, matrix2)






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
        robot_assigned = False 




rospy.init_node('controller3')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(1)


sub = rospy.Subscriber('/estimated_pose', MarkerPose, callback)
rospy.spin()