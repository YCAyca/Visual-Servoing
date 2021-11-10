import rospy
from vs_project.msg import MarkerPose
from vs_project.msg import detectedMarker
import numpy as np
from geometry_msgs.msg import Twist
import sys

ROBOT_MARKER_ID = 0
TARGET_MARKER_ID = 3  
target_assigned = False
robot_assigned = False
target_reached = False


Z = -0.001002 # dont know what to put now just putted the robot z pose in gazebo sim

uv_robot = []
uv_target = []
interaction_matrix = []
error = []

def controller(robot, target, alpha=0.7):
    global interaction_matrix
    global error
    global target_reached
    interaction_matrix.clear()
    error.clear()
   
    for i in range(3):  # using only 3 corners not 4
        error.append(robot[i][0] - target[i][0])
        error.append(robot[i][1] - target[i][1])

    calculate_interaction_matrix(robot)
   
    error_np = np.array(error).reshape(6,1)
    interaction_matrix_np = np.array(interaction_matrix).reshape(6,6)

    interaction_matrix_np = np.linalg.inv(interaction_matrix_np) #inverse of the interaction matrix

    print("error",error_np)

    error_norm = np.linalg.norm(error_np)

    robot_vel = Twist()

    if error_norm < 100:
        robot_vel.linear.x = 0
        robot_vel.linear.y = 0
        robot_vel.linear.z = 0

        robot_vel.angular.x = 0
        robot_vel.angular.y = 0
        robot_vel.angular.z = 0
        
        pub.publish(robot_vel)
        print("robot reached th target ciaou")
        rospy.signal_shutdown("robot reached th target ciaou")
        


    print("error norm", error_norm)
    print("interaction matrix",interaction_matrix_np)

    
    robot_velocity = -alpha * np.matmul(interaction_matrix_np, error_np)



    print("ROBOT VELOCITY", robot_velocity) 

    robot_vel.linear.x = robot_velocity[0][0]
    robot_vel.linear.y = robot_velocity[1][0]
    # robot_vel.linear.z = robot_velocity[2][0]

    # robot_vel.angular.x = robot_velocity[3][0]
    # robot_vel.angular.y = robot_velocity[4][0]
    robot_vel.angular.z = robot_velocity[5][0]

    pub.publish(robot_vel)



def calculate_interaction_matrix(robot):
    for i in range(3):
      interaction_matrix.append([-1/Z, 0, robot[i][0]/Z,  robot[i][0]*robot[i][1], -(1 + robot[i][0]**2),  robot[i][1]])
      interaction_matrix.append([0, -1/Z, robot[i][1]/Z,  1 + robot[i][1]**2,  -robot[i][0]*robot[i][1], -robot[i][0]])  
  

def callback(msg):   
    global target_assigned
    global robot_assigned   

    if msg.id == ROBOT_MARKER_ID:
     #   print("robot uv") 
        uv_robot.clear()
        uv_robot.append((msg.corner1.x, msg.corner1.y))
        uv_robot.append((msg.corner2.x, msg.corner2.y))
        uv_robot.append((msg.corner3.x, msg.corner3.y))
        robot_assigned = True
      #  uv_robot.append((msg.corner4.x, msg.corner4.y))
    elif msg.id == TARGET_MARKER_ID:  # no need to assign target possition again and again
        #   print("target uv") 
     #   if not target_assigned:
        uv_target.clear()
        uv_target.append((msg.corner1.x, msg.corner1.y))
        uv_target.append((msg.corner2.x, msg.corner2.y))
        uv_target.append((msg.corner3.x, msg.corner3.y))
    #  uv_target.append((msg.corner4.x, msg.corner4.y))
        target_assigned = True
    else:
        raise ValueError("check your marker IDs")    

    if target_assigned and robot_assigned:
        controller(uv_robot, uv_target)
        robot_assigned = False # we should be sure if the new robot position came before calling controller serially
        target_assigned = False # no need if the target doesnt move
        


rospy.init_node('controller')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10)


sub = rospy.Subscriber('/detected_marker', detectedMarker, callback)
rospy.spin()

  