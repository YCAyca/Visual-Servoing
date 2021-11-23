import cv2
import numpy as np
from PIL import Image, ImageDraw
import math
import astar

env = cv2.imread("/home/yca/catkin_ws/src/vs_project/env.jpg")

h,w,channel = env.shape

print(h,w)

cv2.imshow("initial env", env)
cv2.waitKey(0)

# find the colors within the specified boundaries and applY  the mask

lower = np.array([0,0,245]) # b,g,r values
upper = np.array([20,20,255]) # b,g,r values

mask = cv2.inRange(env, lower, upper)
output = cv2.bitwise_and(env, env, mask = mask)


for i in range(w):
    for k in range(h):
        if output[k][i].any() == 0: 
            output[k][i] = 255

grayscale = cv2.cvtColor(output, cv2.COLOR_RGB2GRAY)

print(grayscale)

cv2.imshow("map grayscale", grayscale)
cv2.waitKey(0)

im_bw = cv2.threshold(grayscale, 127, 1, cv2.THRESH_BINARY)[1]   
im_bw = im_bw.astype(dtype='f') 
     
# Draw grid lines for visualization

y_start = 0
y_end = h
step_size = 50

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

print("width of maze", maze_w)
print("height of maze", maze_h)

for i in range(0,w, step_size):
    for k in range(0,h, step_size):
        for j in range(step_size):
            if k+j >= h or i+j >= w:
                continue
            window_area.append(im_bw[k+j][i+j])
        if 0 in window_area:
            maze[int(k/step_size)][int(i/step_size)] = 0
        else:
            maze[int(k/step_size)][int(i/step_size)] = 1    
        window_area.clear()    

#maze = np.array(maze, dtype='f')

print("maze", np.array(maze))

cv2.imshow("maze", np.array(maze, dtype='f'))
cv2.waitKey(0)

for i in range(maze_h):
    for k in range(maze_w):
        if maze[i][k] == 0:
            maze[i][k] = 1
        else:
            maze[i][k] = 0

print("maze inversed",np.array(maze))                


start = (1, 9) # robot pos TO DO : update with real robot pos (not pose but the pixel coordinates as the center of 4 detected corner of robot marker than /step_size for x and y)
end = (7, 0) # target pos TO DO : update with real robot pos (not pose but the pixel coordinates as the center of 4 detected corner of robot marker than /step_size for x and y)

maze = np.array(maze, dtype='f')

maze = cv2.circle(maze, start, radius=0, color=(255,0, 0), thickness=-1)

maze = cv2.circle(maze, end, radius=0, color=(255,0, 0), thickness=-1)

cv2.imshow("robot and target pos", maze)
cv2.waitKey(0)

path = astar.astar(maze, start, end)
print(path)

path_draw = [[0 for x in range(maze_w)] for y in range(maze_h)]

for i,k in path:
    path_draw[k][i] = 255

cv2.imshow("calculated path", np.array(path_draw, dtype='f'))
cv2.waitKey(0)

path_real = list()

for i,k in path:
    path_real.append(((i+1)*50,k*50))

print(path_real)


for index in path_real:
    cv2.circle(grayscale, index, radius=1, color=(255,0, 0), thickness=10)

cv2.imshow("real path grayscale", grayscale)
cv2.waitKey(0)  

for index in path_real:
    cv2.circle(env, index, radius=1, color=(255,0, 0), thickness=10)

cv2.imshow("real path orj image", env)
cv2.waitKey(0)      

cv2.destroyAllWindows()    

# TODO : convert these waypoints to the camera frame poses, determine each way point as target point, go to the target points 1 by 1 




