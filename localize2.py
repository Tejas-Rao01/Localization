#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 19 18:06:19 2023

@author: shreyash
"""

# -*- coding: utf-8 -*-
"""
Created on Sat Dec 31 14:45:02 2022

@author: Tejas Rao 
"""

import robot_params 
import math
#import numpy as np
import localization_constants
import matplotlib.pyplot as plt
import ekf


def localize(lidar_data, odometry_data, robotX, robotY, robotTheta,unrobotX, unrobotY, unrobotTheta, P):
    
    [lidarTime, robotX_actual, robotY_actual, robotTheta_actual, \
        num_points, scanning_angle, start_angle, step_size, lidar_data] \
        = process_lidar_data(lidar_data)
    
    [odomTime, SL, SR] = odometry_data
    
    
    [robotX_bar, robotY_bar, robotTheta_bar] = get_pred_pos(SL, SR, robotX, robotY, robotTheta)    
    [unrobotX, unrobotY, unrobotTheta] = get_pred_pos(SL, SR, unrobotX, unrobotY, unrobotTheta)    
    [cylinders, derivatives] = find_cylinders(robotX_bar, robotY_bar, robotTheta_bar, lidar_data)
    
    #plot_scan(lidar_data, derivatives)
    if len(cylinders) > 0:
        [pred_cyl_coords, actual_cyl_coords] = get_cylinder_pairs(cylinders)
        plot_actual_map()
        
        robotX, robotY, robotTheta, P = ekf.kalman_filter([robotX, robotY, robotTheta], [pred_cyl_coords, actual_cyl_coords], P, SL, SR)
# =============================================================================
#         print('localized robot: ', [robotX, robotY, robotTheta])
# =============================================================================
        
        
        return   [robotX, robotY, robotTheta, unrobotX, unrobotY, unrobotTheta, P]#, lidar_world]

    else:
        return  [robotX_bar, robotY_bar, robotTheta_bar, unrobotX, unrobotY, unrobotTheta, P]#, lidar_world]
    


def get_pred_pos(SL, SR,  robotX, robotY, robotTheta):
    
    
    b = robot_params.pioneer_track_width
    
    delta_trans = (SL + SR) / 2 
    #delta_theta = (SR- SL) / (2 * b)
    
    robotX = robotX + delta_trans * math.cos(robotTheta + (SR- SL) / (2 * b))
    robotY = robotY + delta_trans * math.sin(robotTheta + (SR- SL) / (2 * b))
    
    robotTheta = robotTheta + (SR - SL) / ( b)
    
    return [robotX, robotY, robotTheta]

def plot_scan(lidar_data, derivatives):
    x = []
    y = []
    xder= []
    yder = []
    
    plt.figure()
    for i in range(len(lidar_data)):
        
        x.append(i)
        y.append(lidar_data[i][1])
        
        if i > 0 and i < len(lidar_data)-1:
            xder.append(i)
            yder.append(derivatives[i-1])
    
    plt.plot(x, y)
    plt.plot(xder, yder)
    plt.pause(0.005)
    plt.clf()
    
def plot_cylinders(cylinders):
    
    for i in range(len(cylinders)):
        
        plt.scatter(cylinders[i][0], cylinders[i][1], c = 'blue', marker = 'x')
    
def plot_actual_map():

    cylinders = localization_constants.world_cylinders
    ab = localization_constants.arena_bottom
    at = localization_constants.arena_top
    al = localization_constants.arena_left
    ar = localization_constants.arena_right
    
    
    # plot cylinders
    
    for i in range(len(cylinders)):
        x = cylinders[i][0]
        y = cylinders[i][1]
        
        plt.scatter(x, y, linewidths=10, c = 'red')
    
    #plot walls 
    plt.plot([al, ar], [at, at], c = 'red')
    plt.plot([al, ar], [ab, ab] ,c = 'red')
    plt.plot([al, al], [ab, at], c = 'red')
    plt.plot([ar, ar], [ab, at], c = 'red')
        
def plot_world_coords(X, Y, colour):

    plt.scatter(X, Y, c = colour, linewidths=0.1)

def get_cylinder_pairs(cylinders):
    cylinder_pairs = []
    pred_cyl_coords = []
    actual_cyl_coords = []
    world_cylinders = localization_constants.world_cylinders
    
   
    for i in range(len(cylinders)):
        
        min_dist = localization_constants.cylinder_threshold
        cyl_coords = []
        
        for j in range(len(world_cylinders)):
            
            dist = get_dist(cylinders[i][0:2], world_cylinders[j]) 
            
            if dist < min_dist:
                min_dist = dist 
                cyl_coords = world_cylinders[j]
        if cyl_coords:
            pred_cyl_coords.append(cylinders[i])
            actual_cyl_coords.append(cyl_coords)
    
    cylinder_pairs = [pred_cyl_coords, actual_cyl_coords]
    
    return cylinder_pairs
                
def get_dist(cylinder, world_cylinder):
    
    dist = math.sqrt( (cylinder[0] - world_cylinder[0])**2 + (cylinder[1] - world_cylinder[1])**2 )
    
    return dist
    
def get_world_coords(robotX, robotY, robotTheta, lidar_data):
    
    coords = []
    coordsx = []
    coordsy = []

    for i in range(len(lidar_data)):
        
        
        angle = robotTheta + lidar_data[i][0] 
        
        #print(angle * 180/ math.pi)
        r = lidar_data[i][1]
        x = robotX + r * math.cos(angle)
        y = robotY + r * math.sin(angle)
        
        coords.append([x, y])
        coordsx.append(x)
        coordsy.append(y)
              
    return coords, coordsx, coordsy
        
def subsample_lidar_data(lidar_data):
    
    subsampling_rate = localization_constants.lidar_subsample_rate
    subsampled_lidar_data = []
    
    i = 0 
    while i < len(lidar_data):
        subsampled_lidar_data.append(lidar_data[i])
        i += subsampling_rate
    
    return subsampled_lidar_data

def compute_derivative(lidar_data):
        
    # Computing Derivatives 
    derivative = []
    for i in range(1,len(lidar_data)-1):
        l = lidar_data[i-1][1]
        r = lidar_data[i+1][1]
        d = (r- l)/2
        derivative.append(d)
          
    return derivative

def find_cylinders(robotX, robotY, robotTheta, lidar_data):
    
    derivative = compute_derivative(lidar_data)
    cylinders = []
    start = False
    for i in range(len(derivative)):

        if derivative[i] < -localization_constants.cylinder_threshold_derivative :
            start = True
            avg_angle = 0
            n_indices = 0
            avg_depth = 0
            n_indices = 0
            avg_depth = 0 
            avg_indice = 0
            start = True
        if start == True and derivative[i] > localization_constants.cylinder_threshold_derivative \
            and n_indices > 0:
            avg_indice  = avg_indice / n_indices
            avg_angle = avg_angle / n_indices
            avg_depth = avg_depth / n_indices + localization_constants.cylinder_offset
            if avg_depth> 0.2:
                
                theta = robotTheta + avg_angle
                x = robotX + avg_depth * math.cos(theta)
                y = robotY + avg_depth * math.sin(theta)                
                cylinders.append([x, y, avg_depth, avg_angle])
            
            start = False
        if start == True:
            avg_angle += lidar_data[i+1][0]
            avg_indice += i
            n_indices += 1
            avg_depth += lidar_data[i+1][1]
        
    
    return [cylinders, derivative]

def process_lidar_data(data):
    
    sysTime = data[1]
    
    robotX_actual = data[2]
    robotY_actual = data[3]
    robotTheta_actual = data[4]
    
    num_points = int(data[5])
    scanning_angle = data[6]
    start_angle = data[7]
    
    step_size = data[8]
    
    lidar_data = data[9:]
    
    lidar_data_processed = []
    angle = start_angle
    #print('num points: ', num_points)
    for i in range(num_points):
        
        lidar_data_processed.append([angle, lidar_data[i]])
    #print('len of lidardata processed: ', len(lidar_data_processed))
        angle+= step_size
    return [sysTime, robotX_actual, robotY_actual, robotTheta_actual, \
        num_points, scanning_angle, start_angle, step_size, lidar_data_processed]
    

    
    
    
    


