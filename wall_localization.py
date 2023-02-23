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

def localize(lidar_data, odometry_data, robotX, robotY, robotTheta,unrobotX, unrobotY, unrobotTheta, P):
    
    
    [lidarTime, robotX_actual, robotY_actual, robotTheta_actual, \
        num_points, scanning_angle, start_angle, step_size, lidar_data] \
        = process_lidar_data(lidar_data)
  
    [odomTime, SL, SR] = odometry_data
        
    [unrobotX, unrobotY, unrobotTheta] = get_pred_pos(SL, SR, unrobotX, unrobotY, unrobotTheta)    

    
    robotX, robotY, robotTheta, P = wall_ekf.kalman_filter([robotX, robotY, robotTheta], lidar_data, P, SL, SR)
    return  [robotX, robotY, robotTheta, unrobotX, unrobotY, unrobotTheta, P]#, lidar_world]
    


def get_pred_pos(SL, SR,  robotX, robotY, robotTheta):
    
    
    b = robot_params.pioneer_track_width
    
    delta_trans = (SL + SR) / 2 
    
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
    
    coords = [][[0, 5.174055576324463], [0.043633230961859226, 5.17898416519165], [0.08726646192371845, 5.193819046020508], [0.13089969288557768, 5.2186784744262695], [0.1745329238474369, 5.253850936889648], [0.21816615480929613, 5.299656867980957], [0.26179938577115536, 5.3565545082092285], [0.3054326167330146, 5.4251251220703125], [0.3490658476948738, 5.506093502044678], [0.39269907865673304, 5.600335597991943], [0.43633230961859226, 5.708916664123535], [0.4799655405804515, 5.833094120025635], [0.5235987715423107, 5.974440574645996], [0.5672320025041699, 6.134780406951904], [0.6108652334660292, 6.316312313079834], [0.6544984644278884, 6.521710395812988], [0.6981316953897476, 6.754204273223877], [0.7417649263516068, 7.017646312713623], [0.7853981573134661, 7.317074298858643], [0.8290313882753253, 2.476125955581665], [0.8726646192371845, 2.408474922180176], [0.9162978501990438, 2.4038805961608887], [0.959931081160903, 2.4546027183532715], [1.0035643121227622, 7.55621862411499], [1.0471975430846214, 7.3587470054626465], [1.0908307740464807, 6.424011707305908], [1.1344640050083399, 7.031696796417236], [1.178097235970199, 6.897967338562012], [1.2217304669320583, 6.781896114349365], [1.2653636978939176, 6.682178497314453], [1.3089969288557768, 6.597728252410889], [1.352630159817636, 6.527655601501465], [1.3962633907794952, 6.471245288848877], [1.4398966217413545, 6.427933692932129], [1.4835298527032137, 6.397292137145996], [1.527163083665073, 6.3790283203125], [1.5707963146269321, 6.372963905334473], [1.6144295455887914, 5.633415699005127], [1.6580627765506506, 5.610532760620117], [1.7016960075125098, 6.427979946136475], [1.745329238474369, 6.471309185028076], [1.7889624694362283, 6.527737140655518], [1.8325957003980875, 6.597827911376953], [1.8762289313599467, 6.682297229766846], [1.919862162321806, 6.7820353507995605], [1.9634953932836652, 6.898125171661377], [2.0071286242455244, 2.679995536804199], [2.0507618552073836, 2.6354498863220215], [2.094395086169243, 2.6562340259552], [2.138028317131102, 2.776843547821045], [2.1816615480929613, 7.780105113983154], [2.2252947790548205, 7.434841156005859], [2.2689280100166798, 7.0412774085998535], [2.312561240978539, 6.699395179748535], [2.356194471940398, 6.400798797607422], [2.3998277029022574, 6.138817310333252], [2.4434609338641167, 5.908288478851318], [2.487094164825976, 5.704913139343262], [2.530727395787835, 5.525240421295166], [2.5743606267496943, 5.3664445877075195], [2.6179938577115536, 5.22619104385376], [2.661627088673413, 5.102524757385254], [2.705260319635272, 4.99388313293457], [2.7488935505971313, 4.8989033699035645], [2.7925267815589905, 3.8938357830047607], [2.8361600125208497, 3.8578662872314453], [2.879793243482709, 3.980001449584961], [2.923426474444568, 4.635887145996094], [2.9670597054064274, 4.595820426940918], [3.0106929363682866, 4.5650529861450195], [3.054326167330146, 4.543264389038086], [3.097959398292005, 4.5302886962890625], [3.1415926292538643, 4.525976657867432]]
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