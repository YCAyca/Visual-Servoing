import cv2
import numpy as np
from PIL import Image
import astar
import utils

ROBOT_MARKER_ID = 0
TARGET_MARKER_ID = 1 

def map():

    target_matrices = []
    utils.init()

    """ detect robot, target and obstacles pixelwise """

    proj_m, calib_m, dist_coefs = utils.read_calibration_file("ost_real.yaml")

    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    arucoParams = cv2.aruco.DetectorParameters_create() 

    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(calib_m, dist_coefs, (3,3), 1, (3,3))
    
    env = cv2.imread(utils.ENV_IMAGE_NAME)

    (corners, ids, rejected) = cv2.aruco.detectMarkers(env, arucoDict,parameters=arucoParams)

    if len(corners) > 0:
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2)) 
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(env, (cX, cY), 4, (0, 0, 255), -1)
            # draw the ArUco marker ID on the image
            
            if markerID == ROBOT_MARKER_ID:
                cv2.putText(env,  "R", (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                global robot_indexes
                robot_indexes = (cX,cY)

            elif markerID == TARGET_MARKER_ID:
                cv2.putText(env, "T", (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2) 
                global target_indexes
                target_indexes = (cX,cY)  


    h,w,channel = env.shape

    print(h,w)

    cv2.imshow("initial env", env)
    cv2.waitKey(0)


    """ create the grid map having robot target and obstacles """

    lower = np.array([0,0,255]) # b,g,r values
    upper = np.array([255,255,255]) # b,g,r values

    mask = cv2.inRange(env, lower, upper)
    output = cv2.bitwise_and(env, env, mask = mask)


    for i in range(w):
        for k in range(h):
            if output[k][i].any() == 0: 
                output[k][i] = 255

    grayscale = cv2.cvtColor(output, cv2.COLOR_RGB2GRAY)

    im_bw = cv2.threshold(grayscale, 127, 1, cv2.THRESH_BINARY)[1]   
    im_bw = im_bw.astype(dtype='f') 
        
    # Draw grid lines for visualization

    y_start = 0
    y_end = h
    step_size = 40

    for x in range(0, w, step_size):
        cv2.line(grayscale, (x, y_start), (x, y_end), (0, 255, 0), thickness=1)
    

    x_start = 0
    x_end = w  

    for y in range(0, h, step_size):
        cv2.line(grayscale, (x_start, y), (x_end, y), (0, 255, 0), thickness=1)    

    cv2.imshow("grid map", grayscale)
    cv2.waitKey(0)    

    window_area = list()
    maze = [[0 for x in range(int(w/step_size)+1)] for y in range(int(h/step_size)+1)]

    maze_h = len(maze)
    maze_w = len(maze[0])

    """ create the maze to apply A start algorithm on it """

    print("width of maze", maze_w)
    print("height of maze", maze_h)

    for i in range(0,w, step_size):
        for k in range(0,h, step_size):
            for j in range(step_size):
                if k+j >= h or i+j >= w:
                    continue
                window_area.append(im_bw[k+j][i+j])
            if 0 in window_area:
                maze[int(k/step_size)][int(i/step_size)] = 1
            else:
                maze[int(k/step_size)][int(i/step_size)] = 0    
            window_area.clear()    

    print("maze", np.array(maze))

    maze = np.array(maze)

    maze_t = maze.T

    print("robot indexes", robot_indexes)
    print("target indexes", target_indexes)

    start = (int(robot_indexes[0]/step_size), int(robot_indexes[1]/step_size)) 
    end = (int(target_indexes[0]/step_size), int(target_indexes[1]/step_size)) 

    maze = np.array(maze, dtype='f')

    maze = cv2.circle(maze, start, radius=0, color=(255,0, 0), thickness=-1)

    maze = cv2.circle(maze, end, radius=0, color=(255,0, 0), thickness=-1)

    cv2.imshow("maze with robot and target positions", maze)
    cv2.waitKey(0)   

    """ apply A start algorithm """

    path = astar.astar(maze_t, start, end)
    print(path)

    path_draw = [[0 for x in range(maze_w)] for y in range(maze_h)]

    for i,k in path:
        path_draw[k][i] = 255

    cv2.imshow("calculated path", np.array(path_draw, dtype='f'))
    cv2.waitKey(0)     

    """ visualize the calculated path """

    path_real = list()

    for i,k in path:
        path_real.append(((i+1)*step_size,k*step_size))

    print(path_real)


    for index in path_real:
        cv2.circle(grayscale, index, radius=1, color=(255,0, 0), thickness=10)

    for index in path_real:
        cv2.circle(env, index, radius=1, color=(255,0, 0), thickness=10)

    cv2.imshow("real path orj image", env)
    cv2.waitKey(0)      

    """ Convert the calculated path - waypoints to the real time target points """

    marker = Image.open(utils.MARKER_IMAGE_NAME)
    env2 = Image.open(utils.ENV_IMAGE_NAME)

    for waypoint in path_real:
        env2.paste(marker, (waypoint[0],waypoint[1]))
    

    env2 = np.array(env2)

    cv2.imshow("way points as markers", env2)
    cv2.waitKey(0) 

    (corners, ids, rejected) = cv2.aruco.detectMarkers(env2, arucoDict,parameters=arucoParams)

    if len(corners) > 0:
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.15, calib_m, dist_coefs)  
            Rmat_target = utils.rot_matrix(rvec)
            homogenous_target = utils.homogenous_matrix(Rmat_target, tvec[0][0])   
            target_matrices.append(homogenous_target)  
            # Draw Axis
            cv2.aruco.drawAxis(env2, calib_m, dist_coefs, rvec, tvec, 0.1)        
            
    cv2.imshow("pose estimated markers", env2)
    cv2.waitKey(0) 
    cv2.destroyAllWindows() 

    return target_matrices
