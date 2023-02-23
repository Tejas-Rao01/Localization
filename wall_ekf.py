#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 23 10:51:45 2023

@author: shreyash
"""


import localization_constants 
import robot_params
import numpy as np 
import math
import matplotlib.pyplot as plt
import wall_detection

# Redefine robotPos to be a np array

def kalman_filter(robotPos, lidar_data, P, SL, SR):
    

    # Unpacking inputs
    [robotX, robotY, robotTheta] = robotPos
    

    # Differences 
    delta_D = (SR- SL ) / 2
    delta_theta = (SR - SL) / robot_params.pioneer_track_width
    

    # get covariance matrices 
    Q = get_Q(SL, SR)    #delta_theta = (SR- SL) / (2 * b)
    Fp = get_Fp(delta_D, robotTheta, delta_theta)
    Fu = get_Fu(delta_D, robotTheta, delta_theta)
    R = localization_constants.R
    

    [robotX_bar, robotY_bar, robotTheta_bar] = get_pred_pos(SL, SR, robotX, robotY, robotTheta)
    
    Pbar = np.matmul(np.matmul(Fp, P), np.transpose(Fp)) + np.matmul(np.matmul(Fu, Q), np.transpose(Fu))
    
    wall_detector = wall_detection.wall_detection(lidar_data, robotX, robotY, robotTheta)
    detected_walls = wall_detector.detected_walls(flag = False)
    
    if len(detected_walls) == 0:
        return robotX_bar, robotY_bar, robotTheta_bar, Pbar
    
    innovation_cov =  get_inn_cov()
    K_z = np.zeros([3,1])
    K_h = np.zeros([3, 3])

    for i in range(len(detected_walls)):
        
        
        
        
        zi = np.array([detected_walls[0], detected_walls[1]])
        innovation,H = get_corresponding_wall(zi, robotX_bar, robotY_bar, robotTheta_bar)

        K = compute_kalman_gain(Pbar, H, R) 
    
        if innovation[0,0] > math.pi:
            innovation[0,0] = 2 * math.pi - innovation[0,0]
            
        if innovation[0,0] < -math.pi:
            innovation[0,0] = -(2 * math.pi - abs(innovation[0,0]))
        

        K_z = np.add(K_z, np.matmul(K,innovation))
        K_h = np.add(K_h, np.matmul(K,H ))

    [robotX, robotY, robotTheta] = update_pos(robotX_bar, robotY_bar, robotTheta_bar, 1/(len(detected_cyl_coords)) *K_z)
    P = update_uncertainity(Pbar,1/(len(detected_cyl_coords))* K_h)
    
    robotTheta = robotTheta % (2 * math.pi)
    if robotTheta > math.pi:
        robotTheta = robotTheta - math.pi * 2
    if robotTheta < -math.pi:
        robotTheta = robotTheta + math.pi * 2 

    
    plot_pred_cyls(robotX, robotY, robotTheta, detected_cyl_coords, 'blue')
    
    return [robotX, robotY, robotTheta, P]



def plot_pred_cyls(robotX, robotY, robotTheta, detected_cyl_coords, color):
    
    for i in range(len(detected_cyl_coords)):
        x = robotX + detected_cyl_coords[i][2] * math.cos(detected_cyl_coords[i][3] + robotTheta - math.pi/2)
        y =robotY + detected_cyl_coords[i][2] * math.sin(detected_cyl_coords[i][3] + robotTheta - math.pi/2)
        plt.scatter(x, y, c = color)
        


def check_the_bloody_shit(robotX_bar, robotY_bar, robotTheta_bar, detected_cyl_coords):
    
    x_pred = detected_cyl_coords[0][0]
    y_pred = detected_cyl_coords[0][1]
    
    x = robotX_bar + detected_cyl_coords[0][2] * math.cos(detected_cyl_coords[0][3] -math.pi/2 + robotTheta_bar)
    y = robotY_bar + detected_cyl_coords[0][2] * math.sin(detected_cyl_coords[0][3] -math.pi/2 + robotTheta_bar)

def get_pred_pos(SL, SR,  robotX, robotY, robotTheta):
    
    
    b = robot_params.pioneer_track_width
    
    delta_trans = (SL + SR) / 2 
    #delta_theta = (SR- SL) / (2 * b)
    
    robotX = robotX + delta_trans * math.cos(robotTheta + (SR- SL) / (2 * b))
    robotY = robotY + delta_trans * math.sin(robotTheta + (SR- SL) / (2 * b))
    
    robotTheta = robotTheta + (SR - SL) / ( b)
    
    return [robotX, robotY, robotTheta]

def update_uncertainity(Pbar, K_h):
    I= np.eye(3)
    T = np.subtract(I, K_h)
    P = np.matmul(T, Pbar)
    return P
    
def update_pos(robotX_bar, robotY_bar, robotTheta_bar,K_z):
    
    
    robotPos = np.array([[robotX_bar], [robotY_bar], [robotTheta_bar]])

    robotPos = np.add(robotPos, K_z)
    
    return [robotPos[0][0],robotPos[1][0], robotPos[2][0]]
    
    

def compute_kalman_gain(Pbar, H, R):
    
    T =   np.linalg.inv(    np.add(np.matmul(np.matmul(H, Pbar), np.transpose(H)), R))
    K = np.matmul( np.matmul(Pbar, np.transpose(H)), T)
    
    return K 


def get_Fp(delta_D, theta, delta_theta):
    
    Fp = np.array(  [[1, 0 , -delta_D * math.sin(theta + delta_theta / 2)], \
                     [0, 1, delta_D * math.cos(theta + delta_theta / 2)], \
                     [0, 0, 1] ]   )
    
    return Fp

def get_Fu(delta_D, theta, delta_theta):
# =============================================================================
#     print('delta D: ', delta_D)
#     print('deltat Theta: ', delta_theta)
# =============================================================================
    L = robot_params.pioneer_track_width
    
    Fu = np.array(  [[1/2 * math.cos(theta + delta_theta /2) - delta_D / (2 * L) * math.sin(theta + delta_theta / 2), 1/2 * math.cos(theta + delta_theta /2) + delta_D / (2 * L) * math.sin(theta + delta_theta / 2)       ], \
                     [1/2 * math.sin(theta + delta_theta /2) + delta_D / (2 * L) * math.cos(theta + delta_theta / 2), 1/2 * math.sin(theta + delta_theta /2) - delta_D / (2 * L) * math.cos(theta + delta_theta / 2)       ], \
                     [1 / L, -1/L]    ] ) 
        
    return Fu


def get_obs(cyl_coords, robotX, robotY, robotTheta):
    
    dx  = cyl_coords[0] - robotX
    dy = cyl_coords[1] - robotY     
    dist = math.sqrt((cyl_coords[0] - robotX)**2 + (cyl_coords[1] - robotY)**2)
    alpha = math.atan2(dy, dx) - robotTheta + math.pi / 2 
    if alpha < -math.pi:
        alpha = 2 * math.pi + alpha
    if alpha > math.pi:
        alpha = alpha - math.pi * 2 
    
    return [np.array([[dist], [alpha]]), (dx, dy)]


def get_Q(SL, SR):
    k = localization_constants.model_cov_const
    Q = np.array([[k * abs(SR), 0], [0 , k * abs(SL)]])
    return Q        


def world2robot(alpha, r, robotX, robotY, robotTheta):
    
    return [alpha + math.pi/2 - robotTheta, r - (robotX * np.cos(alpha) + robotY * np.sin(alpha))]

def get_corresponding_wall(zi, robotX_bar, robotY_bar, robotTheta_bar):
    alpha_pred, r_pred = zi 
    for i in range(len(localization_constants.world_walls)):
        [alpha_world, r_world] = localization_constants.world_walls[i]
        [alpha, r] = world2robot(alpha_world, r_world, robotX_bar, robotY_bar, robotTheta_bar)
        if alpha < 0:
            alpha = 2*math.pi + alpha
        alpha_measured, r_measured = zi 
        
        if (alpha - alpha_measured < math.pi/180 * 5) and (r - r_measured) < 0.2:
            H = np.array([[0, 0, -1],   [-np.cos(alpha), -np.sin(alpha), 0]])
            innovation = np.array([[alpha_measured - alpha], [r_measured - r]])
            return innovation, H
                    
    return None

    
    