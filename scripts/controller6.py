import cv2
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import time
from geometry_msgs.msg import Twist  
import matplotlib.pyplot as plt

robotpath = "/home/yca/catkin_ws/src/vs_project/images/target7.png"
targetpath = "/home/yca/catkin_ws/src/vs_project/images/robot1.png"

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


proj_m, calib_m, dist_coefs = read_calibration_file("ost_real.yaml")
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
arucoParams = cv2.aruco.DetectorParameters_create() 
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(calib_m, dist_coefs, (3,3), 1, (3,3))

robotimg = cv2.imread(robotpath, cv2.IMREAD_UNCHANGED)
targetimg = cv2.imread(targetpath, cv2.IMREAD_UNCHANGED)

""" read initial pose images """

if robotimg is not None:
    cv2.imshow('robot', robotimg)
    cv2.waitKey(0) 
else:
    print( "robot path is wrong")

if targetimg is not None:
    cv2.imshow('target', targetimg)
    cv2.waitKey(0) 
else:
    print( "target path is wrong")    


cv2.destroyAllWindows()    

""" estimate robot's pose"""

robotimg = cv2.undistort(robotimg, calib_m, dist_coefs, newCameraMatrix=newcameramtx)

(corners, ids, rejected) = cv2.aruco.detectMarkers(robotimg, arucoDict,parameters=arucoParams)

if len(corners) > 0:
    for (markerCorner, markerID) in zip(corners, ids):
        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.1, calib_m, dist_coefs)  

        # Draw Axis
        cv2.aruco.drawAxis(robotimg, calib_m, dist_coefs, rvec, tvec, 0.1)  
        cv2.imshow("robot estimated pose", robotimg)
        cv2.waitKey(0)

        Rmat_robot = rot_matrix(rvec[0][0])
        global homogenous_robot
        homogenous_robot = homogenous_matrix(Rmat_robot, tvec[0][0])

        print("homogenous robot", np.array(homogenous_robot))

        r = R.from_rotvec(rvec[0][0])
        euler_degrees = r.as_euler('xyz', degrees=True)
        print(euler_degrees)


        """ write rvec and tvec to the robot.txt"""

        with open('robotpos.txt', 'w') as outfile:
            outfile.write("Rotation Vector: \n")
            outfile.write(str(rvec[0][0]))
            outfile.write("\n") 
            outfile.write("Translation Vector: \n") 
            outfile.write(str(tvec[0][0])) 
            outfile.write("\n")  
            outfile.write("Homogenous Matrix Robot: \n") 
            outfile.write(str(homogenous_robot))
            outfile.write("\n")  
            outfile.write("Teta Robot: \n") 
            outfile.write(str(euler_degrees[2]))
            outfile.close()  

""" estimate target pose """        

targetimg = cv2.undistort(targetimg, calib_m, dist_coefs, newCameraMatrix=newcameramtx)

(corners, ids, rejected) = cv2.aruco.detectMarkers(targetimg, arucoDict,parameters=arucoParams)

if len(corners) > 0:
    for (markerCorner, markerID) in zip(corners, ids):
        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.1, calib_m, dist_coefs)  

        # Draw Axis
        cv2.aruco.drawAxis(targetimg, calib_m, dist_coefs, rvec, tvec, 0.1)  
        cv2.imshow("target estimated pose", targetimg)
        cv2.waitKey(0)

        Rmat_target = rot_matrix(rvec[0][0])
        global homogenous_target
        homogenous_target = homogenous_matrix(Rmat_target, tvec[0][0])

        print("homogenous target", np.array(homogenous_target))

        r = R.from_rotvec(rvec[0][0])
        euler_degrees = r.as_euler('xyz', degrees=True)

        """ write rvec and tvec to the target.txt"""

        with open('targetpos.txt', 'w') as outfile:
            outfile.write("Rotation Vector: \n")
            outfile.write(str(rvec[0][0]))
            outfile.write("\n")  
            outfile.write("Translation Vector: \n") 
            outfile.write(str(tvec[0][0]))
            outfile.write("\n")  
            outfile.write("Homogenous Matrix Target: \n") 
            outfile.write(str(homogenous_target))
            outfile.write("\n")  
            outfile.write("Teta Target: \n") 
            outfile.write(str(euler_degrees[2]))
            outfile.close()  

cv2.destroyAllWindows()           


""" robot to target pose calculation """

homogenous_robot_inv = np.linalg.inv(homogenous_robot)

Trobotgoal = np.matmul(homogenous_robot_inv,homogenous_target)

deltax =  Trobotgoal[0][3]
deltay =  Trobotgoal[1][3]

print("deltax", deltax)
print("deltay", deltay)

rotation_matrix = [[Trobotgoal[0][0], Trobotgoal[0][1], Trobotgoal[0][2]], [Trobotgoal[1][0], Trobotgoal[1][1], Trobotgoal[1][2]], [Trobotgoal[2][0], Trobotgoal[2][1], Trobotgoal[2][2]]]    

r = R.from_matrix(rotation_matrix)

euler_degrees = r.as_euler('xyz', degrees=True)

print("THETA ROBOT GOAL", euler_degrees[2])

alfa = np.arctan2(deltay, deltax)

print("ALFA", np.degrees(alfa))


with open('robot_to_target.txt', 'w') as outfile:
    outfile.write("Homogenous Matrix: \n") 
    outfile.write(str(Trobotgoal))
    outfile.write("\n")  
    outfile.write("Theta: \n") 
    outfile.write(str(euler_degrees[2]))
    outfile.close()  


""" move the robot """

import rospy

#rospy.init_node('controller6')
#pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

robot_vel = Twist()

robot_vel.linear.x = 0
robot_vel.linear.y = 0
robot_vel.linear.z = 0

robot_vel.angular.x = 0
robot_vel.angular.y = 0
robot_vel.angular.z = 0

robot_x = 0
robot_y = 0
robot_teta = 0

target_x = deltax
target_y = deltay

k_alfa = 1
k_rho = 0.3

t_k = time.time()

plt.ion()
fig, ax = plt.subplots()
plot_points = ax.scatter([], [])
plt.grid()
plt.xlabel("X coordinate")
plt.ylabel("Y coordinate")
it = 0

while True:
    point = np.array([[robot_x, robot_y]])
    array = plot_points.get_offsets()
    array = np.append(array, point, axis=0)
    plot_points.set_offsets(array)
    ## We change the axis limits to see al the points
    ax.set_xlim(array[:, 0].min() - 0.5, array[:,0].max() + 0.5)
    ax.set_ylim(array[:, 1].min() - 0.5, array[:, 1].max() + 0.5)
    plt.title("Trajectory plot\nIteration #: {}".format(it))
    it += 1
    fig.canvas.draw()
    ## We require this line to let the plot update
    plt.pause(0.001)


    distance = math.sqrt((target_x-robot_x)**2 + (target_y-robot_y)**2)

    if distance < 0.1:
        print("Robot reached the target!")
        robot_vel.linear.x = 0
        robot_vel.angular.z = 0
        plt.ioff()
        plt.close()
    #    pub.publish(robot_vel)
    
     #   rospy.signal_shutdown("robot reached th target ciaou")
        break
  
    if math.fabs(np.degrees(alfa)) > 5:
        speed_v = 0
        speed_w = k_alfa * alfa 
    
        robot_vel.linear.x = speed_v
        robot_vel.angular.z = speed_w
    
     #   pub.publish(robot_vel)
        time.sleep(0.001)
        current_time = time.time()
        delta_t = current_time -  t_k
        robot_teta = robot_teta + (speed_w * delta_t)
        t_k = current_time = time.time()
        alfa = np.arctan2((target_y - robot_y), (target_x - robot_x)) - robot_teta
           
        print("alfa",np.degrees(alfa))

    else:
        speed_v = distance * k_rho
        speed_w = 0
        
        robot_vel.linear.x = speed_v
        robot_vel.angular.z = speed_w
    
       # pub.publish(robot_vel)
        time.sleep(0.001)
        current_time = time.time()
        delta_t = current_time -  t_k
        robot_teta = robot_teta + speed_w * delta_t
        robot_x = robot_x + (speed_v * math.cos(robot_teta))*delta_t
        robot_y = robot_y + (speed_v * math.sin(robot_teta))*delta_t
        t_k = current_time = time.time()
       
    print("robot x", robot_x)
    print("robot y", robot_y)
