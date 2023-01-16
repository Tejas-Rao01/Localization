# -*- coding: utf-8 -*-
"""
Created on Mon Dec 12 18:29:40 2022

@author: Tejas Rao
"""
import math
import sim 
import sim_params
import robot_params
import numpy as np

class Controller():
    
    def __init__(self, client_ID, goals, handles):       
        self.client_ID = client_ID
        self.goals = goals 
        self.e_prev = 0
        self.e_total = 0
        self.handles = handles 
        
        
        
    def set_vel(self,V, W):
        
        client_ID = self.client_ID
        
        L = robot_params.pioneer_track_width
        R = robot_params.pioneer_wheel_radius
        
        w = max(min(W, robot_params.pioneer_max_W), -1.0*robot_params.pioneer_max_W)
        v = max(min(V, robot_params.pioneer_max_V), -1.0*robot_params.pioneer_max_V)    
        
        Vr = (2*v + w*L) / (2 * R)
        Vl = (2 * v - w * L) / (2 * R) 
        #print('Vr, Vl', Vr, Vl)
    
        # Set velocity 
        
        #print(self.handles.left_motor_handle, self.handles.right_motor_handle)
        
        
        sim.simxSetJointTargetVelocity(client_ID, self.handles.left_motor_handle, Vl, sim.simx_opmode_oneshot)  
        sim.simxSetJointTargetVelocity(client_ID, self.handles.right_motor_handle, Vr, sim.simx_opmode_oneshot)  
        return 
        
    
    def get_goal(self,goal_handle):
        
        client_ID = self.client_ID
        res, goalPosition = sim.simxGetObjectPosition(client_ID, goal_handle, -1, sim.simx_opmode_blocking)
        x = goalPosition[0]
        y = goalPosition[1]
        
        return [x, y]
            
    
    def get_curr_pose(self):
        client_ID = self.client_ID
        #print(self.handles.pioneer_handle)
        res, currPosition = sim.simxGetObjectPosition(client_ID,self.handles.pioneer_handle , -1, sim.simx_opmode_blocking) #self.handles.pioneer_handle
        res, currOrientation = sim.simxGetObjectOrientation(client_ID, self.handles.pioneer_handle, -1, sim.simx_opmode_blocking)
        #print(res, currPosition)
        x = currPosition[0]
        y = currPosition[1]
        theta = currOrientation[2]
        
        return [x, y, theta]
        
        
    def reachedGoal(self, currPos, goalPos):
        #print(currPos)
        [x_goal, y_goal] = goalPos    
        [x_curr, y_curr, theta_curr]  = currPos
        
        dist = math.sqrt( (x_goal - x_curr)**2 + (y_goal - y_curr)**2 )
        
        if dist < 0.3:
            return True
        return False 
    
    
    
    
    def gtg(self, currPos, goalPos):
          
        #Controller parameters
        Kp = 0.00656*2
        Kd = 0.0001
        Ki = 0.0
        dt = 0.5
        #Using PID controller
        
        currX = currPos[0]
        currY = currPos[1]
        currT = currPos[2]
        
        goalX = goalPos[0]
        goalY = goalPos[1]
        
        
        
        delta_theta = np.arctan2( -(currY - goalY) , -(currX - goalX) ) - currT
        
        #Restricting the range
        
        delta_theta = ((delta_theta + np.pi) %  (2*np.pi)) - np.pi
        
                       
        e_new = math.degrees(delta_theta)
        e_dot = e_new - self.e_prev/dt 
        self.e_total = (e_new*dt) + self.e_total*dt
        
        
        W = Kp * e_new + Kd *  e_dot + Ki * self.e_total
        
        self.e_prev  = e_new
        
            #find distance to goal
        d = np.sqrt(((currX - goalX)**2) + ((currY - goalY)**2))
        
        #velocity parameters
        distThresh = 0.1#mm
        
        #control input for linear velocity
        V = (5*0.12/1.5)*(np.arctan(d - distThresh))
        

        return [V, W]
        
    
    def move(self, client_ID, goal):
        
        goalPos = self.get_goal(self.handles.goal_handles[goal])
        currPos = self.get_curr_pose()
        
        [V,W] = self.gtg(currPos, goalPos)   
        #print('V, W',V, W)
        self.set_vel(V, W)
        if self.reachedGoal(currPos, goalPos):
            goal += 1
    
        return [goal, currPos]
        
        
        
        
        
        
        
        
        
            
        
        
        
