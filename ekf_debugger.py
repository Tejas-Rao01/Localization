#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 23 17:33:34 2023

@author: shreyash
"""
import numpy as np
import time 
import wall_detection
import wall_ekf
import wall_localization

file = open('/home/shreyash/Desktop/slam/scan.txt', 'r')
lidar_data = []
for f in file.readlines():
    lidar_point = list(map(float, f.strip().split(' ')))
    lidar_data.append(lidar_point)



robotX, robotY, robotTheta = [1.2, -2.15, 0]
robotPos = [robotX, robotY, robotTheta]
SL, SR = 0,0
P = np.eye(3)

# Checking wall detection
# Todo - fix detection 
wall_detector = wall_detection.wall_detection(lidar_data, robotX, robotY, robotTheta)
detected_walls = wall_detector.detected_walls(False)

# =============================================================================
# wall_detection.plot_line_segments(detected_walls)
# =============================================================================



#  checking ekf for 0 input
for i in range(1):
    [robotX, robotY, robotTheta, P] = wall_ekf.kalman_filter(robotPos, lidar_data, P, SL, SR)
print(robotX)
print(robotY)
print(robotTheta)



