#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 23 10:18:48 2023
@author: shreyash
"""



import robot_params 
import math
#import numpy as np
import localization_constants
import matplotlib.pyplot as plt
import wall_ekf
import wall_detection
import os

def localize(lidar_data, odometry_data, robotX, robotY, robotTheta,unrobotX, unrobotY, unrobotTheta, P):
    
    
    [lidarTime, robotX_actual, robotY_actual, robotTheta_actual, \
        num_points, scanning_angle, start_angle, step_size, lidar_data] \
        = process_lidar_data(lidar_data)
  
    [odomTime, SL, SR] = odometry_data
        
    [unrobotX, unrobotY, unrobotTheta] = get_pred_pos(SL, SR, unrobotX, unrobotY, unrobotTheta)    

    robotX, robotY, robotTheta, P = wall_ekf.kalman_filter([robotX, robotY, robotTheta], lidar_data, P, SL, SR)
    plot_actual_map()
    X, Y = get_world_coords(robotX, robotY, robotTheta, lidar_data)
    plot_world_coords(X, Y, 'red')
    return  [robotX, robotY, robotTheta, unrobotX, unrobotY, unrobotTheta, P]#, lidar_world]
    


def get_pred_pos(SL, SR,  robotX, robotY, robotTheta):
    
    
    b = robot_params.pioneer_track_width
    
    delta_trans = (SL + SR) / 2 
    
    robotX = robotX + delta_trans * math.cos(robotTheta + (SR- SL) / (2 * b))
    robotY = robotY + delta_trans * math.sin(robotTheta + (SR- SL) / (2 * b))
    robotTheta = robotTheta + (SR - SL) / ( b)
    
    return [robotX, robotY, robotTheta]


    

def plot_actual_map():

    cylinders = localization_constants.world_cylinders
    ab = localization_constants.arena_bottom
    at = localization_constants.arena_top
    al = localization_constants.arena_left
    ar = localization_constants.arena_right
    
    
    #plot walls 
    plt.plot([al, ar], [at, at], c = 'red')
    plt.plot([al, ar], [ab, ab] ,c = 'red')
    plt.plot([al, al], [ab, at], c = 'red')
    plt.plot([ar, ar], [ab, at], c = 'red')
        
def plot_world_coords(X, Y, colour):

    plt.scatter(X, Y, c = colour, linewidths=0.1)

                
def get_dist(cylinder, world_cylinder):
    
    dist = math.sqrt( (cylinder[0] - world_cylinder[0])**2 + (cylinder[1] - world_cylinder[1])**2 )
    
    return dist
    
def get_world_coords(robotX, robotY, robotTheta, lidar_data):
    
    coordsx = []
    coordsy = []

    for i in range(len(lidar_data)):
        
        
        angle = robotTheta + lidar_data[i][0] - math.pi/2
# =============================================================================
#         if lidar_data[i] == Inf:
#             continue
# =============================================================================
        #print(angle * 180/ math.pi)
        r = lidar_data[i][1]
        x = robotX + r * math.cos(angle)
        y = robotY + r * math.sin(angle)
    
        coordsx.append(x)
        coordsy.append(y)
              
    return coordsx, coordsy
        


def process_lidar_data(data):
    
    sysTime = data[1]
    
    robotX_actual = data[2]
    robotY_actual = data[3]
    robotTheta_actual = data[4]
    
    num_points = int(data[5])
    scanning_angle = data[6]
    start_angle = 0
    
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