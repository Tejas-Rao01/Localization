# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

## This file is to create false lidar data and try out hough transforms
#hp_WEGQEbCeXuu8qiLXhHP24B8jQe7hIH1bIcWW

import random 
import matplotlib.pyplot as plt
import numpy as np 
import math

# Defining the constants for the walsl 

world_left = -5
world_up = 5
world_down = -5
world_right = 5

std =0.01

bysf = 10
sf = 1/ bysf
# Defining the Walls 
left_wall = [[world_left* (1 + np.random.normal(0,std)), y*sf * (1 + np.random.normal(0,std))] for y in range(world_down*bysf, world_up*bysf, 1)]
right_wall = [[world_right* (1 + np.random.normal(0,std)), y*sf* (1 + np.random.normal(0,std))] for y in range(world_down*bysf, world_up*bysf, 1)]
top_wall = [[x*sf* (1 + np.random.normal(0,std)), world_up * (1 + np.random.normal(0,std))] for x in range(world_left*bysf, world_right*bysf, 1)]
bottom_wall = [[x*sf* (1 + np.random.normal(0,std)), world_down * (1 + np.random.normal(0,std))] for x in range(world_left*bysf, world_right*bysf, 1)]


def plot_wall(wall):
    
    n_pts = len(wall)
    for i in range(n_pts):
        x, y = wall[i]
        plt.scatter(x, y, c= 'blue')
        
    
#Plotting the walls 

# =============================================================================
# plot_wall(left_wall)
# plot_wall(right_wall)
# plot_wall(top_wall)
# plot_wall(bottom_wall)
# 
# =============================================================================

#Implementing the hough Transform
def hough_transform(points, hough_img):
    
    sampling_rate = 0.1
    angle_increment = math.pi / (180 * sampling_rate)
    for point in points:
        #print(point)
        theta = 0
        x, y = point
        for t in range(0, int(180*2* sampling_rate)):
            d = math.sqrt(x**2 + y ** 2)
            alpha = math.atan2(y, x)
            if alpha < 0:
                alpha = 180 + (180 + alpha)
            delta = alpha - theta
            r = d * math.cos(delta)
            theta += angle_increment
            theta_ind = t_val_discrete(theta)
            ind = r_val_discrete(r)
            #print(ind, r)
            hough_img[theta_ind, ind] += 1
    return hough_img
num_t_indices = 60
t_step = 180*2 / num_t_indices
sampling_rate = 1                
hough_img = np.zeros(shape=(num_t_indices, 50))

def t_val_discrete(t):
    num_t_indices = 60
    t_step = math.pi * 2 / num_t_indices 
    print(t_step)
    ind = int(t // t_step)
    return ind
    
def r_val_discrete(r):    
    num_r_indices = 50
    r_step = 10/ num_r_indices
    ind = int(r// r_step)
    return ind

hough_img = hough_transform(left_wall, hough_img)
hough_img = hough_transform(right_wall, hough_img)
hough_img = hough_transform(top_wall, hough_img)
hough_img = hough_transform(bottom_wall, hough_img)
print(hough_img)
plt.imshow(hough_img)


indices =np.argwhere(hough_img > 45)
print(indices)

#plt.clf()
for i in range(len(indices)):
    a, b = indices[i]
    
    theta = a  / 60 * 360 * math.pi / 180
    r = b / 50 * 10 
    
    print(r, theta)
    
    
    
    
    
    