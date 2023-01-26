# -*- coding: utf-8 -*-
"""
Created on Thu Jan 26 09:28:24 2023

@author: Tejas Rao
"""
import math
import numpy as np
import matplotlib.pyplot as plt
import ekf


def plt_robot_pos(robotPos, color):
    
     x = robotPos[0]
     y = robotPos[1]
     theta = robotPos[2]
     
     l = 0.4
     
     plt.scatter(x,y, c =color,marker='s')
     plt.plot([x, x + l * math.cos(theta)], [y, y + l * math.sin(theta)])
     


robotPos = [ 1.5259552941277548, -1.9244834315547794, 0.11540733946861334 ]
robotPos_real = [1.1920775175094604, -2.1522185802459717, 0.09460326284170151]
# =============================================================================
# detected_cyl_coords = [[2*math.cos(math.pi/10), 2*math.sin(math.pi/10), 2, math.pi/2], [2*math.cos(math.pi/10 + math.pi/2), 2*math.sin(math.pi/2 + math.pi/10), 2, math.pi], [math.sqrt(2)*math.cos(math.pi/10 + math.pi/4), math.sqrt(2)*math.sin(math.pi/4 + math.pi/10), math.sqrt(2), math.pi/4 + math.pi/2]]
# actual_cyl_coords = [[2, 0], [0,2], [1,1]]
# 
# =============================================================================
detected_cyl_coords =  [[3.874125184075278, -0.5072862303388028, 2.742690237557016, 1.9984019780531526], [2.524647848076071, 2.04045005702758, 4.0887754140076815, 2.7794368122704327]]
print(math.sqrt((robotPos[0] - detected_cyl_coords[0][0])**2 + (robotPos[1] - detected_cyl_coords[0][1])**2))

actual_cyl_coords = [[3.5252, -0.8], [2.2497, 1.7498]]
cylinder_pairs = [detected_cyl_coords, actual_cyl_coords]

plt.xlim(-3, 8)
plt.ylim(-8, 3)


# Plot the cylinders
for i in range(len(cylinder_pairs[0])):
    #plt.scatter(cylinder_pairs[0][i][0], cylinder_pairs[0][i][1], c='blue')
    plt.scatter(cylinder_pairs[1][i][0], cylinder_pairs[1][i][1], c='red')

# Plot the coordinates of the robot 

plt_robot_pos(robotPos, 'blue')
plt_robot_pos(robotPos_real,'red')


SL = 0
SR = 0 
P = np.eye(3)*0.5

[robotX, robotY, robotTheta, P] = ekf.kalman_filter(robotPos, cylinder_pairs, P, SL, SR)

plt_robot_pos([robotX, robotY, robotTheta], 'green')

ax = plt.gca()
ax.set_aspect('equal', adjustable='box')
plt.draw()




## testing individual funcitons 

## Testing the Kalman Gain Matrix 

R = np.eye(2)
H = np.array([[1, 0 , -1], [0, -1, 1]])
Pbar = np.array([[1,-1,0],[-1,1,0],[0,0,1]])
K = ekf.compute_kalman_gain(Pbar, H, R)
print('K: ', K)


## Testing the 


