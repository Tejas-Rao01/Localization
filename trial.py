#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb  3 20:06:19 2023

@author: shreyash
"""                    
import numpy as np
import matplotlib.pyplot as plt 
import wall_detection
import wall_detection_efficient
import datetime
import time 
lidar_scan = np.load('/home/shreyash/Desktop/slam/lidar_scan.npy')
print(lidar_scan.shape)

# Plot just one time step 


scan = lidar_scan[0, :]



data = []

plt.xlim(-10.5,10)
plt.ylim(-8.5, 8)
for scan_id in range(20,100):
    data = []
    scan = lidar_scan[scan_id, :]
    X = []
    Y = []
    min_angle= scan[0]
    max_angle = scan[1]
    angle_increment = scan[2]
    angle = min_angle
    
    T0 = datetime.datetime.now()
    for i in range(3, lidar_scan.shape[1]):
        
        r = scan[i]
        if r > 10:
            angle += angle_increment    
            continue

# =============================================================================
#             print(angle, r)
# =============================================================================
        data.append([angle, r])
        x = r * np.cos(angle)
        y = r * np.sin(angle)
        angle += angle_increment
        
        X.append(x)
        Y.append(y)

# =============================================================================
#     num_ticks = 10
#     tick = time.time()
#     wall_detector1 = wall_detection_efficient.wall_detection(scan[3:], 0, 0, 0, min_angle, max_angle, angle_increment)
#     wall_detector1.debug(num_ticks)
#     tock = time.time()
#     
#     dt1 = tock - tick 
#     
#     tick = time.time()
#     wall_detector2 = wall_detection.wall_detection(data, 0, 0, 0)
#     wall_detector2.debug(num_ticks)
# 
# 
#     tock = time.time()
#     dt2 = tock - tick 
#     print(f'compile time for regular: {dt2}')
#     print(f'compile time for efficient: {dt1}')
# =============================================================================
    print(len(data))
    tick = time.time()    
    wall_detector = wall_detection.wall_detection(data, 0, 0, 0)
    line_segments = wall_detector.detected_walls(flag = False)
    wall_detection.plot_line_segments(line_segments)
    tock = time.time()
    plt.pause(0.00001)
    print(f'tick {tick}')
    print(f'tock {tock}')
    print(f'time taken for iter {scan_id} = {tock -tick}')

    del wall_detector
                                                                                                                                                                               