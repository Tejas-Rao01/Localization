# -*- coding: utf-8 -*-
"""
Created on Tue Dec 13 17:29:20 2022

@author: Tejas Rao
"""

import matplotlib.pyplot as plt
import math
import time
import numpy as np
import robot_params
import random

# Lidar data contains several lidar scans
# Lidar scan --> [angle, distance]
def process_data(): 
    f = open('scan.txt')
    lines = f.readlines()
    lidar_data = []
    #print(lines)
    for l in lines:
        l = l.split(' ')
        lidar_scan = []
        i = 0
        #print(len(l))
        while i < len(l)-1:    
           
            r = float(l[i+1])
            if r < 0.012:
                r = 10
            lidar_scan.append([float(l[i]), r]) 
            i += 2         
        
        lidar_data.append(lidar_scan)
        
    return lidar_data 

def compute_derivative(lidar_data):
    
    # Obtaining the lidar data each scan
    # Obtain range and scan index
    RayIndices = []
    Ranges = []
    Derivatives = []
    for lidar_scan in lidar_data:
        ScanRayIndices = []
        ranges = []
        for i in range(len(lidar_scan)):
            ScanRayIndices.append(i)
            ranges.append(lidar_scan[i][1])
        RayIndices.append(ScanRayIndices)
        Ranges.append(ranges)
        
        # Computing Derivatives 
        derivative = []
        for i in range(1,len(lidar_scan)-1):
            l = lidar_scan[i-1][1]
            r = lidar_scan[i+1][1]
            d = (r- l)/2
            derivative.append(d)
            
        Derivatives.append(derivative)
        
    return [Derivatives, Ranges, RayIndices]

# Returns the detected cylinders for each iteration
def find_cylinders(lidar_data):
    
    [Derivatives, Ranges, RayIndices] = compute_derivative(lidar_data)
        
    threshold = 0.1
    Cylinders = []
    
    for d in range(len(Derivatives)):
        ranges = Ranges[d]
        derivative = Derivatives[d]
        cylinders = []
        start = True
        avg_indice = 0
        n_indices = 0
        avg_depth = 0
        for i in range(len(derivative)):
            cylinder_offset = 0.3   
            if derivative[i] < -threshold :
                start = True
                avg_indice = 0
                n_indices = 0
                avg_depth = 0
                n_indices = 0
                avg_depth= 0 
                avg_indice = 0
                start = True
            if start == True and derivative[i] > threshold and n_indices > 0:
                avg_indice  = avg_indice / n_indices
                avg_depth = avg_depth / n_indices + cylinder_offset
                if avg_depth> 0.2:
                    cylinders.append([avg_depth, avg_indice])
                
                #if avg_indice < 25 and d == 0:
                    #print(i, derivative[i-10:i+10])
                start = False
            if start == True:
                avg_indice += i
                n_indices += 1
                avg_depth += ranges[i+1]
        
        Cylinders.append(cylinders)
    return [Derivatives, Ranges, RayIndices,Cylinders]

def rayInd2angle(ind, num_indices):
    total_ind = num_indices
    angle = ind * math.pi / (total_ind-1) 
    return angle
    
    
def plot_lidar_data(Derivatives,Ranges, RayIndices, Cylinders):
    
    scan_id = 0
    cylinders = Cylinders[scan_id]
    cyl_ray = []
    cyl_dist = []
    for i in range(len(cylinders)):
        cyl_ray.append(cylinders[i][1])
        cyl_dist.append(cylinders[i][0])
        
    ranges = Ranges[scan_id]
    ray_indices = RayIndices[scan_id]
    
    plt.plot(ray_indices, ranges)
    plt.plot(ray_indices[1:-1], Derivatives[0])
    plt.scatter(cyl_ray, cyl_dist)
    #plt.pause(0.1)
    #plt.clf()

def transform_coordinates(robot_x, robot_y, robot_theta, X, Y):
    theta = robot_theta - math.pi/2
    x_tf = []
    y_tf = []
    for i in range(len(X)):
        
        x_dash = X[i]
        y_dash = Y[i]
        x = x_dash * math.cos(theta) - y_dash * math.sin(theta) 
        y = x_dash * math.sin(theta) + y_dash * math.cos(theta) 
        
        x = x + robot_x
        y = y + robot_y
        
        x_tf.append(x)
        y_tf.append(y)
        
    return [x_tf, y_tf]

def get_cyl_pairs(pred_coords, actual_coords):
    
    threshold = 0.3 
    correspondences = []
    for i in range(len(pred_coords)):
        
        pred_cylX = pred_coords[i][0]
        pred_cylY = pred_coords[i][1]
        
        cyl = -1 
        min_dist = 100000
        for j in range(len(actual_coords)):
            
            act_cylX = actual_coords[j][0]
            act_cylY = actual_coords[j][1]
            
            dist = math.sqrt((pred_cylX - act_cylX)**2 \
                             + (pred_cylY - act_cylY)**2)
            
            if dist < threshold and dist < min_dist:
                min_dist = dist
                cyl = j
        if cyl > -1:
            correspondences.append([i, cyl])
    return correspondences
        
def plot_scan_2D(Ranges, RayIndices, actual_pos, Cylinders, cyl_actual_coordinates):
    
    # Obtain actual cylinder coordinates
    cyl_actual_coordinatesX = cyl_actual_coordinates[0]
    cyl_actual_coordinatesY = cyl_actual_coordinates[1]
    Cyl_act = []
    #print(len(Ranges), len(actual_pos))
    actual_coords = []
    for i in range(len(cyl_actual_coordinates)):       
        actual_coords.append([cyl_actual_coordinates[0][i],cyl_actual_coordinates[1][i] ])
    
    for scan_id in range(len(Ranges)):
        
        robot_x_old = actual_pos[scan_id][0]
        robot_y_old = actual_pos[scan_id][1]
        robot_theta_old =  actual_pos[scan_id][2]
        
        ranges = Ranges[scan_id]
        ray_indices = RayIndices[scan_id]
        num_indices = len(ray_indices)
        #
        #print(num_indices)
        ray_indices = [rayInd2angle(ind, num_indices) for ind in ray_indices]
        X = []
        Y = []
        for r in range(len(ranges)):
            x = ranges[r] * math.cos(ray_indices[r])
            y = ranges[r] * math.sin(ray_indices[r])
            
            X.append(x)
            Y.append(y)
        [X_old, Y_old] = transform_coordinates(robot_x_old, robot_y_old, robot_theta_old, X, Y)
        
        
         
        #Detected Cylinders 
        
        cylinders = Cylinders[scan_id]
        cyl_angles = []
        for i in range(len(cylinders)):
            cyl_angles.append(rayInd2angle(cylinders[i][1], num_indices))
        
        
        cyl_robotX = []
        cyl_robotY = []
        for i in range(len(cyl_angles)):
            r = cylinders[i][0]
            #print(r)
            if r > 0.5:
                cyl_robotX.append(r * math.cos(cyl_angles[i]))
                cyl_robotY.append(r * math.sin(cyl_angles[i]))   
             
        [cyl_worldX, cyl_worldY] = transform_coordinates(robot_x_old, robot_y_old, robot_theta_old, cyl_robotX, cyl_robotY)
        
        detected_coords = []
        for i in range(len(cyl_worldX)):
            detected_coords.append([cyl_worldX[i], cyl_worldY[i]])
            
        correspondences = get_cyl_pairs(detected_coords, actual_coords)       
        [detected_walls, actual_walls] = get_corresponding_points_on_wall(X_old, Y_old)
        
        
        
        trafo = get_transform([detected_coords[pair[0]] for pair in correspondences], \
                             [ actual_coords[pair[1]]for pair in correspondences])
        
            
            
        cyl_act = []
    
        for i in range(len(cyl_worldX)):
            cyl_act.append([cyl_worldX[i],cyl_worldY[i]])
        

        plt.scatter(cyl_actual_coordinatesX, cyl_actual_coordinatesY, c = 'yellow',marker = 'o', linewidths=10)
        print(robot_theta_old * 180 / math.pi)
        
        plt.scatter(cyl_worldX, cyl_worldY, marker = 'x')
        
        
        if trafo:
            print(trafo)
            for i in range(len(detected_coords)):
                
               [correctedX, correctedY] = apply_transform(trafo, detected_coords[i])           
               plt.scatter(correctedX, correctedY,marker =  'v')
            
            [robot_x_new, robot_y_new, robot_theta_new]  = correct_pose([robot_x_old, robot_y_old, robot_theta_old], trafo)     
        else:
            [robot_x_new, robot_y_new, robot_theta_new] = [robot_x_old, robot_y_old, robot_theta_old]
        
        X = []
        Y = []
        for r in range(len(ranges)):
            x = ranges[r] * math.cos(ray_indices[r])
            y = ranges[r] * math.sin(ray_indices[r])
            
            X.append(x)
            Y.append(y)
        [X_new, Y_new] = transform_coordinates(robot_x_new, robot_y_new, robot_theta_new, X, Y)
        
        
        plt.scatter(robot_x_old, robot_y_old, c = 'red', marker = "o")        
        plt.scatter(robot_x_new, robot_y_new, c = 'blue', marker = "o")
        
        
        plt.scatter(X_old, Y_old, c = 'pink', s =2)
        plt.scatter(X_new, Y_new, c = 'green', s =2)
        ## Adding arrow to show direction 
        
        arrow_startX = robot_x_old
        arrow_startY = robot_y_old
        arrow_length = 0.5
        arrow_endX = robot_x_old  + arrow_length * math.cos(robot_theta_old)
        arrow_endY = robot_y_old + arrow_length * math.sin(robot_theta_old)       
        
        
        plt.plot([arrow_startX, arrow_endX], [arrow_startY, arrow_endY])
        
        
        
        plt.xlim(-3, 8)
        plt.ylim(-8,3)
        plt.pause(0.001)
        plt.clf()
        
        
        Cyl_act.append(cyl_act)
    return Cyl_act


    
    for i in range(len(actual_pos)):
        
        theta = actual_pos[i][2]
    
    return theta

def get_pos():
    
    f = open('actual_pos.txt', 'r')
    lines = f.readlines()
    actual_pos = []
    for line in lines:
        #print(line)
        l = list(map(float, line.split(' ')))
        actual_pos.append(l)
    return actual_pos
        
def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)   


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
    
# Given a similarity transformation:
# trafo = (scale, cos(angle), sin(angle), x_translation, y_translation)
# and a point p = (x, y), return the transformed point.
def apply_transform(trafo, p):
    lam, c, s, tx, ty = trafo
    lac = lam * c
    las = lam * s
    x = lac * p[0] - las * p[1] + tx
    y = las * p[0] + lac * p[1] + ty
    return (x, y)

def correct_pose(pose, trafo):
    la, c, s, tx, ty = trafo
    #print 'trafo:', trafo
    old_x = pose[0]
    old_y = pose[1]
    old_theta = pose[2]
    #print 'pose: ', pose
    x, y = apply_transform( trafo, (old_x,old_y) )
    theta = old_theta + np.arctan2(s,c)
    #print 'new pose: ', (x,y,theta)
    return (x, y, theta)  # Replace this by the corrected pose.
    #return(old_x,old_y,theta)


def delta_pos(x,y, theta, motor_ticks_curr, motor_ticks_old):
    
    deltaL = robot_params.pioneer_wheel_radius * (motor_ticks_curr[0] - motor_ticks_old[0])
    deltaR = robot_params.pioneer_wheel_radius * (motor_ticks_curr[1] - motor_ticks_old[1])
    
    Xnew = x + (deltaL + deltaR)/ (2) * math.cos(theta + (deltaR - deltaL)/ (2 * robot_params.pioneer_track_width))
    Ynew = y + (deltaL + deltaR)/ (2) * math.sin(theta + (deltaR - deltaL)/ (2 * robot_params.pioneer_track_width))
    Thetanew = (deltaR - deltaL)/ (2)
    
    return [Xnew, Ynew, Thetanew]


def pred_pos(initial_pos):
    f = open('motor_ticks.txt', 'r')
    
    lines = f.readlines()
    #print(len(lines))
    motor_ticks = []
    for l in lines:
        motor_ticks.append(list(map(float, l.split())))
    
    i = 0
    motor_leftx = motor_ticks[i][0]
    motor_lefty = motor_ticks[i][1]
    motor_rightx = motor_ticks[i][2]
    motor_righty = motor_ticks[i][3]
    
    x_old = (motor_leftx + motor_rightx) / 2 
    y_old = (motor_righty + motor_lefty) / 2
    
    angle = np.arctan2((motor_righty - motor_lefty), (motor_rightx - motor_leftx))
    theta_old = angle + math.pi / 2 
    
    predicted_pos = [[x_old, y_old, theta_old]]

    scale_factor = 0.02
    
    mu = 0 
    sigma = 2
    
    for i in range(1, len(motor_ticks)):
        
        motor_leftx = motor_ticks[i][0]
        motor_lefty = motor_ticks[i][1]
        motor_rightx = motor_ticks[i][2]
        motor_righty = motor_ticks[i][3]
        
        
        x_new = (motor_leftx + motor_rightx) / 2 + random.gauss(mu, sigma) * scale_factor
        y_new  = (motor_righty + motor_lefty) / 2 + random.gauss(mu, sigma) * scale_factor
        angle = np.arctan2((motor_righty - motor_lefty), (motor_rightx - motor_leftx))
        theta_new = angle + math.pi / 2  + random.gauss(mu, sigma) * scale_factor
 
        predicted_pos.append([x_new, y_new, theta_new])

    plt.close()
    
    return predicted_pos
    
    
def get_corresponding_points_on_wall(X, Y,
                                     arena_left = -2.5, arena_right = 7.5,
                                     arena_bottom = -7.5, arena_top = 2.5,
                                     eps = 0.2):
    detected_walls = []
    actual_walls = []
    
    
    
    for i in range(len(X)):
        x = X[i]
        y = Y[i]
        
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
    

def plot_pred_pos(pred_pos, actual_pos):
    
    X = []
    Y = []
    
    X1 = []
    Y1 = []
    for i in range(len(pred_pos)):
        
        x = pred_pos[i][0]
        y = pred_pos[i][1]
        
        x1 = actual_pos[i][0]
        y1 = actual_pos[i][1]
        
        X.append(x)
        Y.append(y)
        
        X1.append(x1)
        Y1.append(y1)
    
    #print(X, Y)
        plt.scatter(X, Y)
        plt.scatter(X1, Y1, c = 'red')
        plt.pause(0.05)
        plt.clf()
        plt.xlim(-3, 8)
        plt.ylim(-8,3)
        
    return
    

if __name__ == '__main__':

    actual_pos = get_pos() 
    predicted_pos = pred_pos(actual_pos[0])
    
    lidar_data = process_data()   
    [Derivatives, Ranges, RayIndices,Cylinders] = find_cylinders(lidar_data)
    
    
    
    time.sleep(2)
    cyl_actual_coordinatesX = [2.2497, 3.5252, 6.2249, 6.7998, 6.8998, 3.0504, 0.2245, -0.6747]
    cyl_actual_coordinatesY = [1.7498, -0.8 , 0.9749, -1.75, -5.2249, -3.800, -6.3251, -2.7]
    
    cyl_actual_coordinates = [cyl_actual_coordinatesX,cyl_actual_coordinatesY]
    
    Reference_cylinders = [[cyl_actual_coordinatesX[i],cyl_actual_coordinatesY[i]] for i in range(len(cyl_actual_coordinatesX))]
    max_radius = 3
    
    plot_scan_2D(Ranges, RayIndices, predicted_pos, Cylinders,cyl_actual_coordinates)
    
    
    
    
   # plot_pred_pos(predicted_pos,actual_pos)
    
    