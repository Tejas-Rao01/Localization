#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 23 10:18:48 2023
@author: shreyash
"""



import robot_params 
import math
import localization_constants
import matplotlib.pyplot as plt
import wall_ekf
import numpy as np

def localize(lidar_data, step_size, odometry_data, robotX, robotY, robotTheta,unrobotX, unrobotY, unrobotTheta, P):
    
    
    lidar_data = process_lidar_data(lidar_data, step_size)
    #plt.scatter(robotX, robotY,c='red', marker='x')
    # Check 
    [odomTime, SL, SR] = odometry_data
        
    [unrobotX, unrobotY, unrobotTheta] = get_pred_pos(SL, SR, unrobotX, unrobotY, unrobotTheta)    

    robotX, robotY, robotTheta, P, walls = wall_ekf.kalman_filter([robotX, robotY, robotTheta], lidar_data, P, SL, SR)
    plot_walls(walls, robotX, robotY, robotTheta)
    #plt.scatter(robotX, robotY, marker='x')
    
    X, Y = get_world_coords(robotX, robotY, robotTheta, lidar_data)
    #plot_world_coords(X, Y, 'red')
    
    plot_vars = [X, Y, robotX, robotY, robotTheta, walls]
    return  [robotX, robotY, robotTheta, unrobotX, unrobotY, unrobotTheta, P, plot_vars]#, lidar_world]

    
def plot_walls(walls, robotX, robotY, robotTheta):
    
    transformation_mat = np.array([ [np.cos(robotTheta), -np.sin(robotTheta), robotX],[np.sin(robotTheta), np.cos(robotTheta), robotY],[0,0,1]])
    for wall in walls:
        p1, p2 = wall
        p1 = list(p1)
        p2 = list(p2)
        p1.append(1)
        p2.append(1)
        p1 = np.matmul(transformation_mat, np.array(p1).reshape((3,1)))
        p2 = np.matmul(transformation_mat, np.array(p2).reshape( (3,1)))
        
        x = [p1[0], p2[0]]
        y = [p1[1], p2[1]]
        plt.plot(x,y, c='black')
       
# =============================================================================
#         plt.xlim(-5, 8)
#         plt.ylim(-10,10)
# =============================================================================

def get_pred_pos(SL, SR,  robotX, robotY, robotTheta):
    
    
    b = robot_params.pioneer_track_width
    
    delta_trans = (SL + SR) / 2 
    
    robotX = robotX + delta_trans * math.cos(robotTheta + (SR- SL) / (2 * b))
    robotY = robotY + delta_trans * math.sin(robotTheta + (SR- SL) / (2 * b))
    robotTheta = robotTheta + (SR - SL) / ( b)
    
    return [robotX, robotY, robotTheta]

        
def plot_world_coords(X, Y, colour):

    

    plt.scatter(X, Y, c = colour, linewidths=0.1)

                
def get_dist(cylinder, world_cylinder):
    
    dist = math.sqrt( (cylinder[0] - world_cylinder[0])**2 + (cylinder[1] - world_cylinder[1])**2 )
    
    return dist
    
def get_world_coords(robotX, robotY, robotTheta, lidar_data):

    coordsx = []
    coordsy = []

    for i in range(len(lidar_data)):
        
        
        angle = robotTheta + lidar_data[i][0]

        r = lidar_data[i][1]
        x = robotX + r * math.cos(angle)
        y = robotY + r * math.sin(angle)
        if angle < math.pi/4:
            plt.scatter(x,y,linewidths=9, c='black')
        coordsx.append(x)
        coordsy.append(y)
              
    return coordsx, coordsy
        


def process_lidar_data(lidar_data, step_size):
    
    lidar_data_processed = []
    #Start angle
    angle = 0
    
    #num of lidar points  
    num_points = len(lidar_data)
    
    for i in range(num_points):        
        r = lidar_data[i]
        if r == np.inf:
            angle += step_size 
            continue
        lidar_data_processed.append([angle, lidar_data[i]])       
        angle+= step_size
    return lidar_data_processed