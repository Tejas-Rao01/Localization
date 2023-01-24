# -*- coding: utf-8 -*-
"""
Created on Sun Jan  1 11:53:14 2023

@author: Tejas Rao 
"""
import numpy as np
# Store all constants needed for localization

cylinder_offset = 0.25
cylinder_threshold_derivative = 0.1
cylinder_threshold = 2 
arena_left = -2.5
arena_right = 7.5
arena_bottom = -7.5
arena_top = 2.5

eps = 0.2

lidar_subsample_rate = 2

world_cylinders = [[2.2497,1.7498], [3.5252, -0.8 ], [6.2249, 0.9749], [6.7998, -1.75], [ 6.8998, -5.2249], [3.0504,-3.800], [0.2245,-6.3251], [-0.6747, -2.7]]
    

## EKF Params 

model_cov_const = 0.05 ## k
R = np.eye(2)*0.001