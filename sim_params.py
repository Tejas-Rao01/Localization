# -*- coding: utf-8 -*-
"""
Created on Mon Dec 12 16:08:55 2022

@author: Tejas Rao
"""

#Robot Params 
pioneer_wheel_radius = 0.0975 #[m] Radius of the wheels on each pioneer
pioneer_track_width = 0.4  #[m] Distance betweent he two wheels of pioneer
pioneer_max_W = 0.3 #[rad/seconds] Maximum angular velocity (omega) of pioneer
pioneer_max_V = 0.7 #[m/seconds] Maximum linear velocity (V) of pioneer
goal_threshold = 0.3 #[m] The threshold distance at whihc robot is declared to be at goal 




#Handles 
left_motor_handle = '/Pioneer1/leftMotor'
right_motor_handle = '/Pioneer1/rightMotor'
goals = ['goal_1','goal_2','goal_3','goal_4','goal_5','goal_6','goal_7','goal_8']
pioneer_handle = 'Pioneer1'
