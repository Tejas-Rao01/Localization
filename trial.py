#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb  3 20:06:19 2023

@author: shreyash
"""
import numpy as np
import matplotlib.pyplot as plt 
import wall_detection

lidar_scan = np.load('/home/shreyash/Desktop/slam/lidar_scan.npy')
print(lidar_scan.shape)

# Plot just one time step 


scan = lidar_scan[0, :]
min_angle= scan[0]
max_angle = scan[1]
angle_increment = scan[2]
angle = min_angle

data = []


for scan_id in range(1):
    scan = lidar_scan[scan_id, :]
    X = []
    Y = []
    for i in range(3, lidar_scan.shape[1]):
        
        r = scan[i]
        if r > 10:
            angle += angle_increment    
            continue
        data.append([angle, r])
        x = r * np.cos(angle)
        y = r * np.sin(angle)
        angle += angle_increment
        
        X.append(x)
        Y.append(y)
    print(data)   
    wall_detector = wall_detection.wall_detection(data, 0, 0, 0)
    line_segments = wall_detector.detected_walls()
    print(line_segments)
    plt.scatter(X, Y, c = 'red')
    plt.xlim(-4.5, 1.5)
    plt.ylim(-2.5, 4)
    plt.pause(0.01)
    plt.clf()
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    plt.draw()
    
    