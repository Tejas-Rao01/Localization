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


def kalman_filter(robotPos, cylinder_pairs, P, SL, SR):
    
    
    # Unpacking inputs
    [robotX, robotY, robotTheta] = robotPos
    [pred_cyl_coords, actual_cyl_coords] = cylinder_pairs
    
    # Differences 
    delta_D = (SR- SL ) / 2
    delta_theta = (SR - SL) / robot_params.pioneer_track_width
    

    # get covariance matrices 
    Q = get_W(SL, SR)
    Fp = get_Fp(delta_D, robotTheta, delta_theta)
    Fu = get_Fu(delta_D, robotTheta, delta_theta)
    R = localization_constants.R
    e
    # Predict
    [robotX_bar, robotY_bar, robotTheta_bar] = get_pred_pos(SL, SR, robotX, robotY, robotTheta)
    Pbar = np.matmul(np.matmul(Fp, P), np.transpose(Fp)) + np.matmul(np.matmul(Fu, Q), np.transpose(Fu))
    
    print('robotX bar: ', robotX_bar)
    print('robotY bar: ', robotY_bar)
    print('robotTheta bar: ', robotTheta_bar)
    print('Pbar.shape: ', Pbar.shape)
    
    
    #Measurements  
    # Iterate over each obvervation and correct pose robotX =

    for i in range(len(pred_cyl_coords)):
        
        #print(pred_cyl_coords[i])
        zi = np.array((pred_cyl_coords[i][2],pred_cyl_coords[i][3])) 
        [zhati, (dx, dy)] = get_obs(actual_cyl_coords[i], robotX_bar, robotY_bar, robotTheta_bar)
        
        print('zi.shape: ', zi.shape)
        print('zhati.shape: ', zhati.shape)e
        
        H = compute_obs_jacobian(zhati[0, 0], dx, dy)     
        K = compute_kalman_gain(Pbar, H, R) 
        
        print('H.shape ', H.shape)
        print('K.shape', K.shape)
        
        [robotX_bar, robotY_bar, robotTheta_bar] = update_pos(robotX_bar, robotY_bar, robotTheta_bar, K, zi, zhati)
        print('robotX bar: ', robotX_bar)
        print('robotY bar: ', robotY_bar)
        print('robotTheta bar: ', robotTheta_bar)
        P = update_uncertainity(Pbar, K, H)
        
    [robotX, robotY, robotTheta] =  [robotX_bar, robotY_bar, robotTheta_bar] 
    
    return [robotX, robotY, robotTheta, P]



def get_pred_pos(SL, SR,  robotX, robotY, robotTheta):
    
    
    b = robot_params.pioneer_track_width
    
    delta_trans = (SL + SR) / 2 
    #delta_theta = (SR- SL) / (2 * b)
    
    robotX = robotX + delta_trans * math.cos(robotTheta + (SR- SL) / (2 * b))
    robotY = robotY + delta_trans * math.sin(robotTheta + (SR- SL) / (2 * b))
    
    robotTheta = robotTheta + (SR - SL) / ( b)
    
    return [robotX, robotY, robotTheta]

def update_uncertainity(Pbar, K, H):
    I= np.eye(3)
    T = np.subtract(I, np.matmul(K, H))
    P = np.matmul(T, Pbar)
    return P
    
def update_pos(robotX_bar, robotY_bar, robotTheta_bar,K, zi, zhati):
    innovation = np.subtract(zi, zhati)
    robotPos = np.array([[robotX_bar], [robotY_bar], [robotTheta_bar]])
    robotPos = np.add(robotPos, np.matmul(K ,innovation))
    
    return [robotPos[0], robotPos[1], robotPos[2]]
    
    

def compute_kalman_gain(Pbar, H, R):
    
    T =   np.linalg.inv(np.matmul(np.matmul(H, Pbar), np.transpose(H)) + R)
    K = np.matmul( np.matmul(Pbar, np.transpose(H)), T)
    
    return K 
    
def compute_obs_jacobian(q, dx, dy):
    
    H=  1/ (q*q) * np.array([[q* dx, -q * dy, 0], [dy, dx, -1]])
    
    return H


def get_Fp(delta_D, theta, delta_theta):
    
    Fp = np.array(  [[1, 0 , -delta_D * math.sin(theta + delta_theta / 2)], \
                     [0, 1, delta_D * math.cos(theta + delta_theta / 2)], \
                     [0, 0, 1] ]   )
    
    return Fp

def get_Fu(delta_D, theta, delta_theta):
    
    L = robot_params.pioneer_track_width
    
    Fu = np.array(  [[1/2 * math.cos(theta + delta_theta /2) - delta_D / (2 * L) * math.sin(theta + delta_theta / 2), 1/2 * math.cos(theta + delta_theta /2) + delta_D / (2 * L) * math.sin(theta + delta_theta / 2)       ], \
                     [1/2 * math.sin(theta + delta_theta /2) + delta_D / (2 * L) * math.cos(theta + delta_theta / 2), 1/2 * math.sin(theta + delta_theta /2) - delta_D / (2 * L) * math.cos(theta + delta_theta / 2)       ], \
                     [1 / L, -1/L]    ] ) 
    return Fu


def get_obs(cyl_coords, robotX, robotY, robotTheta):
    
    #print(cyl_coords[0], robotX, 'cylcoords0, robotX')
    dx  = cyl_coords[0] - robotX
    #print(cyl_coords[1], robotY, 'cylcoords1, robotY')
    dy = cyl_coords[1] - robotY
    #print('dy dx', dy, dx)
    dist = (cyl_coords[0] - robotX)**2 + (cyl_coords[1] - robotY)**2
    alpha = math.atan2(dy, dx) - robotTheta
    return [np.array([[dist], [alpha]]), (dx, dy)]


def get_W(SL, SR):
    k = localization_constants.model_cov_const
    Q = np.array([[k * abs(SR), 0], [0 , k * abs(SL)]])
    return Q
