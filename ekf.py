#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 19 18:12:11 2023

@author: shreyash
"""

import localization_constants 
import robot_params
import numpy as np 
import math
import matplotlib.pyplot as plt

# Redefine robotPos to be a np array

def kalman_filter(robotPos, cylinder_pairs, P, SL, SR):
    

    # Unpacking inputs
    [robotX, robotY, robotTheta] = robotPos
    print('robotX: ', robotX)
    print('robotY: ', robotY)
    print('robotTheta : ', robotTheta)
    [detected_cyl_coords, actual_cyl_coords] = cylinder_pairs
    print("detected cyl coords: ", detected_cyl_coords)
    print("actual_cyl_coords: ", actual_cyl_coords)
    # Differences 
    delta_D = (SR- SL ) / 2
    delta_theta = (SR - SL) / robot_params.pioneer_track_width
    

    # get covariance matrices 
    Q = get_W(SL, SR)
    Fp = get_Fp(delta_D, robotTheta, delta_theta)
    Fu = get_Fu(delta_D, robotTheta, delta_theta)
    R = localization_constants.R
    
    # detectedict
    [robotX_bar, robotY_bar, robotTheta_bar] = get_pred_pos(SL, SR, robotX, robotY, robotTheta)
    
    Pbar = np.matmul(np.matmul(Fp, P), np.transpose(Fp)) + np.matmul(np.matmul(Fu, Q), np.transpose(Fu))
    
    print('robot theta: ', robotTheta)
# =============================================================================
#     print('P: ', P)
#     print('Fp: ', Fp)
#     print('Fu: ', Fu)
# =============================================================================
# =============================================================================
#     print('Q: ', Q)
# =============================================================================
    print('robotX bar: ', robotX_bar)
    print('robotY bar: ', robotY_bar)
    print('robotTheta bar: ', robotTheta_bar)
# =============================================================================
#     print('Pbar: ', Pbar)
#     print('SL: ', SL)
#     print('SR: ', SR)
# =============================================================================
    
    K_z = np.zeros([3,1])
    K_h = np.zeros([3, 3])
# =============================================================================
#     print('actual_cylinder_coords: ', actual_cyl_coords)
#     print('predicted cylinder coords: ', detected_cyl_coords)
# =============================================================================
    plot_pred_cyls(robotX_bar, robotY_bar, robotTheta_bar, detected_cyl_coords, 'y')
    for i in range(len(detected_cyl_coords)):
        
        
        zi = np.array([[detected_cyl_coords[i][2]],[detected_cyl_coords[i][3]] ])
    
        check_the_bloody_shit(robotX_bar, robotY_bar, robotTheta_bar, detected_cyl_coords)
        
        
        plt.plot([detected_cyl_coords[i][0], actual_cyl_coords[i][0]],[detected_cyl_coords[i][1], actual_cyl_coords[i][1]], c='black')        
        [zhati, (dx, dy)] = get_obs(actual_cyl_coords[i], robotX_bar, robotY_bar, robotTheta_bar)

        print('zi: ', zi)
        print('zhati: ', zhati)
        
        H = compute_obs_jacobian(zhati[0, 0], dx, dy)     
        K = compute_kalman_gain(Pbar, H, R) 
        
        print('H: ', H)
        print('K: ', K)
        
        innovation = np.subtract(zi ,zhati)
        print('innovation: ', innovation)
        if innovation[1,0] > math.pi:
            innovation[1,0] = 2 * math.pi - innovation[1,0]
            
        if innovation[1,0] < -math.pi:
            innovation[1,0] = -(2 * math.pi - abs(innovation[1,0]))
        
# =============================================================================
#         print('P: ', P)
# =============================================================================
        print('innovation: ', innovation)
        
        
        K_z = np.add(K_z, np.matmul(K,innovation))
        K_h = np.add(K_h, np.matmul(K,H ))
        
        
    print('K_z: ', K_z)
    print('K_h: ', K_h)

    [robotX, robotY, robotTheta] = update_pos(robotX_bar, robotY_bar, robotTheta_bar, 1/(len(detected_cyl_coords)) *K_z)
    P = update_uncertainity(Pbar,1/(len(detected_cyl_coords))* K_h)
    
    robotTheta = robotTheta % (2 * math.pi)
    if robotTheta > math.pi:
        robotTheta = robotTheta - math.pi * 2
    if robotTheta < -math.pi:
        robotTheta = robotTheta + math.pi * 2 
    print('robotX (updated): ', robotX)
    print('robotY (updated): ', robotY)
    print('robotTheta (updated): ', robotTheta)
    
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
    
    print('x_given, x_calculated: ', x_pred, x)
    print('y_given, y_calculated: ', y_pred, y)

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
    
    return [robotPos[0][0], robotPos[1][0], robotPos[2][0]]
    
    

def compute_kalman_gain(Pbar, H, R):
    
    T =   np.linalg.inv(    np.add(np.matmul(np.matmul(H, Pbar), np.transpose(H)), R))
    K = np.matmul( np.matmul(Pbar, np.transpose(H)), T)
    
    return K 
    
def compute_obs_jacobian(q, dx, dy):
    
    H=  1/ (q*q) * np.array([[-q* dx, -q * dy, 0], [dy, -dx, -1*q*q]])
    
    return H

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


def get_W(SL, SR):
    k = localization_constants.model_cov_const
    Q = np.array([[k * abs(SR), 0], [0 , k * abs(SL)]])
    return Q
