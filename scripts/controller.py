import rospy
from vs_project.msg import MarkerPose
from vs_project.msg import detectedMarker
import numpy as np
from geometry_msgs.msg import Twist

ROBOT_MARKER_ID = 0
TARGET_MARKER_ID = 3  
target_assigned = False
robot_assigned = False

Z = 1 # dont know what to put now

uv_robot = []
uv_target = []
interaction_matrix = []

def controller(robot, target, alpha):
    pixel_velocity = []
    global interaction_matrix
    interaction_matrix.clear()
   
    for i in range(3):  # using only 3 corners not 4
        pixel_velocity.append(robot[i][0] - target[i][0])
        pixel_velocity.append(robot[i][1] - target[i][1])

    calculate_interaction_matrix(robot)
   
    pixel_velocity_np = np.array(pixel_velocity).reshape(6,1)
    interaction_matrix_np = np.array(interaction_matrix).reshape(6,6)

    print("pixel velocity",pixel_velocity_np)
    print("interaction matrix",interaction_matrix_np)

    
    robot_velocity = -alpha * np.matmul(interaction_matrix_np, pixel_velocity_np)

    print("ROBOT VELOCITY", robot_velocity) 

    robot_vel = Twist()
    robot_vel.linear.x = robot_velocity[0][0]
    print("LINEAR X",robot_vel.linear.x)
    robot_vel.linear.y = robot_velocity[1][0]
    print("LINEAR Y",robot_vel.linear.y)
    robot_vel.linear.z = robot_velocity[2][0]

    robot_vel.angular.x = robot_velocity[3][0]
    robot_vel.angular.y = robot_velocity[4][0]
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
        uv_robot.append((msg.corner1.x, msg.corner1.y))
        uv_robot.append((msg.corner2.x, msg.corner2.y))
        uv_robot.append((msg.corner3.x, msg.corner3.y))
        robot_assigned = True
      #  uv_robot.append((msg.corner4.x, msg.corner4.y))
    elif msg.id == TARGET_MARKER_ID:  # no need to assign target possition again and again
        if not target_assigned:
         #   print("target uv") 
            uv_target.append((msg.corner1.x, msg.corner1.y))
            uv_target.append((msg.corner2.x, msg.corner2.y))
            uv_target.append((msg.corner3.x, msg.corner3.y))
        #  uv_target.append((msg.corner4.x, msg.corner4.y))
            target_assigned = True
    else:
        raise ValueError("check your marker IDs")    

    if target_assigned and robot_assigned:
        controller(uv_robot, uv_target, 1)




rospy.init_node('controller')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

while not rospy.is_shutdown(): 
  sub = rospy.Subscriber('/detected_marker', detectedMarker, callback)
  rospy.spin()