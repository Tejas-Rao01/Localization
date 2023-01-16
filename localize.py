# -*- coding: utf-8 -*-
"""
Created on Sat Dec 31 14:45:02 2022

@author: Tejas Rao 
"""

import robot_params 
import math
import numpy as np
import localization_constants
import matplotlib.pyplot as plt

def localize(lidar_data, odometry_data, robotX, robotY, robotTheta,unrobotX, unrobotY, unrobotTheta):
    
    #print('lidar data', lidar_data)

    [lidarTime, robotX_actual, robotY_actual, robotTheta_actual, \
        num_points, scanning_angle, start_angle, step_size, lidar_data] \
        = process_lidar_data(lidar_data)
    
    [odomTime, SL, SR] = odometry_data
    
    if len(lidar_data)> 0:
        print('lidar data valid - 1')
        print(robotX, robotY, robotTheta)
    else:
        return get_pred_pos(SL, SR, robotX, robotY, robotTheta)
    #print(odomTime, SL, SR)
    [robotX, robotY, robotTheta] = get_pred_pos(SL, SR, robotX, robotY, robotTheta)
    [unrobotX, unrobotY, unrobotTheta] = get_pred_pos(SL, SR, unrobotX, unrobotY, unrobotTheta)
    
    print([unrobotX, unrobotY, unrobotTheta] , ' unrobot')
    
    [cylinders, derivatives] = find_cylinders(robotX, robotY, robotTheta, lidar_data)
    [lidar_world, X, Y] = get_world_coords(robotX, robotY, robotTheta, lidar_data)  
    
    #plot_scan(lidar_data, derivatives)
    plot_world_coords(X,Y, 'blue')
    plt.scatter(robotX, robotY, marker = 'v', c = 'blue')            
    plot_cylinders(cylinders)
    
    if len(cylinders) > 0:
        print('cylinders detected - 2')
        print(f'num_cylinders = {len(cylinders)}')
        [pred_cyl_coords, actual_cyl_coords] = get_cylinder_pairs(cylinders)
    else:
        return  [robotX, robotY, robotTheta, unrobotX, unrobotY, unrobotTheta]#, lidar_world]
    
    if len(pred_cyl_coords) > 0 :
        print('cylinder pairs obtained - 3')
        print('num cylinder pairs = ', len(pred_cyl_coords))
    # get cylinder transform and correct
    trafo_cyl = get_transform(pred_cyl_coords, actual_cyl_coords)
    
    
    if trafo_cyl:
        [robotX, robotY, robotTheta] = correct_pos(robotX, robotY, robotTheta, trafo_cyl)   
        print('cylinder transform obtained - 4')
        print('transform: ', trafo_cyl)
    
   
    plot_actual_map()
   
    # Sub-sampled points
    subsampled_lidar_data = subsample_lidar_data(lidar_data)
    subLidar_world = get_world_coords(robotX, robotY, robotTheta, subsampled_lidar_data)
    
    print('lidar data subsampled - 5')
    
# =============================================================================
#     trafo_walls = get_wall_transform(trafo_cyl, robotX, robotY, robotTheta, subLidar_world, subsampled_lidar_data)
#     if trafo_walls:
#         [robotX, robotY, robotTheta] = correct_pos(robotX, robotY, robotTheta, trafo_walls)
#         print('ok 5')
# 
#     #lidar_world = get_world_coords(robotX, robotY, lidar_data)
#     #lidar_world = apply_transform(lidar_world, trafo_walls)
#     
#     [robotX, robotY, robotTheta] = correct_pos(robotX, robotY, robotTheta, trafo_walls)
#     
# =============================================================================
    print('ok - 6')
    return   [robotX, robotY, robotTheta, unrobotX, unrobotY, unrobotTheta]#, lidar_world]


def plot_scan(lidar_data, derivatives):
    x = []
    y = []
    xder= []
    yder = []
    
    for i in range(len(lidar_data)):
        
        x.append(i)
        y.append(lidar_data[i][1])
        
        if i > 0 and i < len(lidar_data)-1:
            xder.append(i)
            yder.append(derivatives[i-1])
    
    plt.plot(x, y)
    plt.plot(xder, yder)
    


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


def get_wall_transform(trafo, robotX, robotY, robotTheta, subLidar_world, subsampled_lidar_data):
    
    if not trafo:
        trafo = [1,1,0, 0, 0]
    overall_trafo = trafo
    iters = localization_constants.icp_iters
    for i in range(iters):
       [detected_walls, actual_walls] = find_walls(subLidar_world)
       trafo = get_transform(detected_walls, actual_walls)
       if trafo:
           overall_trafo = concatenate_transform(trafo, overall_trafo)
       
           [robotX, robotY, robotTheta] = correct_pos(robotX, robotY, robotTheta, trafo)
           subLidar_world = get_world_coords(robotX, robotY, subsampled_lidar_data)
     
    return overall_trafo



def apply_transform(trafo, p):
    lam, c, s, tx, ty = trafo
    lac = lam * c
    las = lam * s
    x = lac * p[0] - las * p[1] + tx
    y = las * p[0] + lac * p[1] + ty
    return (x, y)


def correct_pos(robotX, robotY, robotTheta, trafo):
    la, c, s, tx, ty = trafo
    x, y = apply_transform( trafo, [robotX, robotY])
    theta = robotTheta + np.arctan2(s,c)
    
    return (x, y, theta) 
       
    
def concatenate_transform(a, b):
    laa, ca, sa, txa, tya = a
    lab, cb, sb, txb, tyb = b

    # New lambda is multiplication.
    lag = laa * lab

    # New rotation matrix uses trigonometric angle sum theorems.
    c = ca*cb - sa*sb
    s = sa*cb + ca*sb

    # New translation is a translation plus rotated b translation.
    tx = txa + laa * ca * txb - laa * sa * tyb
    ty = tya + laa * sa * txb + laa * ca * tyb

    return (laa, c, s, tx, ty)
    
    
    
def get_centroid(points):
    
    if len(points) == 0:
        return None
    x_mean = 0
    y_mean = 0 
    
    for i in range(len(points)):
        x_mean += points[i][0]
        y_mean += points[i][1]
    
    x_mean = x_mean / len(points)
    y_mean = y_mean / len(points)

    return [x_mean, y_mean]        
        


def get_transform(detected_coords, actual_coords):
    
    
    m = len(detected_coords)
    
    detected_centroid = get_centroid(detected_coords)
    actual_centroid = get_centroid(actual_coords) 
    
    plt.scatter(detected_centroid[0],detected_centroid[1], c = 'blue', marker = '^')
    plt.scatter(actual_centroid[0], actual_centroid[1], c= 'red', marker = '^')
    
    detected_coords_reduced = []
    actual_coords_reduced = []
    
    for i in range(len(detected_coords)):
        
        reducedX = detected_coords[i][0] - detected_centroid[0]
        reducedY = detected_coords[i][1] - detected_centroid[1]
        
        detected_coords_reduced.append([reducedX, reducedY])
    
    for i in range(len(actual_coords)):
        
        reducedX = actual_coords[i][0] - actual_centroid[0]
        reducedY = actual_coords[i][1] - actual_centroid[1]
        
        actual_coords_reduced.append([reducedX, reducedY])
        
        
    cs,ss,rr,ll = 0.0,0.0,0.0,0.0
    
    
    
    for i in range(m):
        
        cs += detected_coords_reduced[i][0] * actual_coords_reduced[i][0] \
            + detected_coords_reduced[i][1] * actual_coords_reduced[i][1]
        
        ss += -detected_coords_reduced[i][1] * actual_coords_reduced[i][0] \
            + detected_coords_reduced[i][0] * actual_coords_reduced[i][1]
        
        ll += detected_coords_reduced[i][0] * detected_coords_reduced[i][0] \
            + detected_coords_reduced[i][1]*detected_coords_reduced[i][1]
        
        rr += actual_coords_reduced[i][0]* actual_coords_reduced[i][0] \
            + actual_coords_reduced[i][1] * actual_coords_reduced[i][1]


        if rr == 0 or ll == 0:
            return None 
        
        lam = math.sqrt(rr / ll)
        
        if cs == 0 or ss == 0:
            return None 
        
        else: 
            
            c = cs / math.sqrt((cs*cs) + (ss*ss))
            s = ss / math.sqrt((cs*cs) + (ss*ss))
            
        tx = actual_centroid[0] - (lam * ((c * detected_centroid[0]) \
                                          - (s * detected_centroid[1])))
        ty = actual_centroid[1] - (lam * ((s * detected_centroid[0]) \
                                          + (c * detected_centroid[1])))    
        
        return lam, c, s , tx, ty
        



 
    
def get_cylinder_pairs(cylinders):
    cylinder_pairs = []
    pred_cyl_coords = []
    actual_cyl_coords = []
    world_cylinders = localization_constants.world_cylinders
    
   
    for i in range(len(cylinders)):
        
        min_dist = localization_constants.cylinder_threshold
        cyl_coords = []
        
        for j in range(len(world_cylinders)):
            
            dist = get_dist(cylinders[i], world_cylinders[j]) 
            
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



def find_walls(subLidar_world):
    

    wall_pairs = []
    wall_pred_coords = []
    wall_act_coords = []
    
    [arena_bottom, arena_left, arena_right, arena_top] = [localization_constants.arena_bottom,  \
                    localization_constants.arena_left, \
                        localization_constants.arena_right,\
                            localization_constants.arena_top]
        
    eps = localization_constants.eps
    
    detected_walls = []
    actual_walls = []
    
    
    
    for i in range(len(subLidar_world)):
        x = subLidar_world[i][0]
        y = subLidar_world[i][1]
        
        if abs(x - arena_left) < eps:
            detected_walls.append([x, y])
            actual_walls.append([arena_left, y])
            
        elif abs(x - arena_right) < eps:
            detected_walls.append([x, y])
            actual_walls.append([arena_right, y])
            
        elif abs(y- arena_bottom) < eps:
            detected_walls.append([x, y])
            actual_walls.append([x, arena_bottom])
            
        elif abs(y - arena_top) < eps:
            detected_walls.append([x, y])
            actual_walls.append([x, arena_top])
                        
    return [detected_walls, actual_walls]

    
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
        cylinder_offset = 0.3   
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
            avg_depth = avg_depth / n_indices + cylinder_offset
            if avg_depth> 0.2:
                
                theta = robotTheta + avg_angle
                x = robotX + avg_depth * math.cos(theta)
                y = robotY + avg_depth * math.sin(theta)                
                cylinders.append([x, y])
            
            start = False
        if start == True:
            avg_angle += lidar_data[i+1][0]
            avg_indice += i
            n_indices += 1
            avg_depth += lidar_data[i+1][1]
        
    
    return [cylinders, derivative]
        
        
    
def get_pred_pos(SL, SR,  robotX, robotY, robotTheta):
    
    
    b = robot_params.pioneer_track_width
    
    delta_trans = (SL + SR) / 2 
    delta_theta = (SR- SL) / (2 * b)
    
    robotX = robotX + delta_trans * math.cos(robotTheta + (SR- SL) / (2 * b))
    robotY = robotY + delta_trans * math.sin(robotTheta + (SR- SL) / (2 * b))
    
    robotTheta = robotTheta + (SR - SL) / ( b)
    
    return [robotX, robotY, robotTheta]


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
    
        
    
    
    
    
    
    
    
    
    
    
    
    


