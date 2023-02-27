#!/usr/bin/env python

"""
Mobile robot simulation setup
@author: Tejas Rao
"""

#Import libraries
import time

#Import files
import initialize_sim
import controller
import matplotlib.pyplot as plt
import sim
import wall_localization
import robot_params
import numpy as np
import math

def plt_robot_pos(robotPos, color):
    
     x = robotPos[0]
     y = robotPos[1]
     theta = robotPos[2]
     
     l = 0.4
     
     plt.scatter(x,y, c =color,marker='s')
     plt.plot([x, x + l * math.cos(theta)], [y, y + l * math.sin(theta)])
     
     
def main():
    global client_ID
    client_ID = initialize_sim.sim_init()
    if (client_ID > -1):
        
        #Start simulation
        if (initialize_sim.start_simulation(client_ID)):
            
                   
            #initializing the sim params
            goal = 0
            handles = initialize_sim.Handles(client_ID)
            ctrl = controller.Controller(client_ID, goal, handles)
            
            #Stop Robot
            ctrl.set_vel(0,0)
            
            res, lidar_signal  = sim.simxGetStringSignal(client_ID, 'c', sim.simx_opmode_blocking)
            lidar_data = sim.simxUnpackFloats(lidar_signal)
            
            odom_signal = []
            while not odom_signal:
                res, odom_signal = sim.simxGetStringSignal(client_ID, 'odometry', sim.simx_opmode_blocking)
                odometry_data = sim.simxUnpackFloats(odom_signal)
            
            #print('odometry_data: ', odometry_data)
            [time_old, left_vel, right_vel] = odometry_data
            time_old1 = lidar_data[1]
        
            init_pos = ctrl.get_curr_pose()
            [robotX, robotY, robotTheta]  = init_pos
            [unrobotX, unrobotY, unrobotTheta]  = init_pos
            
            currPos_old = [lidar_data[2], lidar_data[3]]
            
            # Store trajectory
            
            act_trajX = []
            act_trajY = []
            
            pred_trajX = []
            pred_trajY = []
            
            unlocalized_trajX = []
            unlocalized_trajY = []
            
            #Initialize Uncertainity Matrix
            P = np.eye(3)
            tick = 0 
            while goal < 5:        #len(handles.goal_handles) :
                
                print('tick: ',tick)
                res, lidar_signal  = sim.simxGetStringSignal(client_ID, 'c', sim.simx_opmode_streaming)
                lidar_data = sim.simxUnpackFloats(lidar_signal)
                
                res, odom_signal = sim.simxGetStringSignal(client_ID, 'odometry', sim.simx_opmode_streaming)
                odometry_data = sim.simxUnpackFloats(odom_signal)
                
                while (not odometry_data):
                    res, odom_signal = sim.simxGetStringSignal(client_ID, 'odometry', sim.simx_opmode_blocking)
                    odometry_data = sim.simxUnpackFloats(odom_signal)
                
                [time_new, left_vel_new, right_vel_new] = odometry_data
                
                dt = time_new - time_old       
                time_old = time_new
                
                SL = dt * left_vel_new * robot_params.pioneer_wheel_radius
                SR = dt * right_vel_new * robot_params.pioneer_wheel_radius 
                
                
                odometry_data = [dt, SL, SR]
                
                if len(lidar_data) > 0 and len(odom_signal) > 0:
                    [goal,_] = ctrl.move(client_ID,goal)
                    #ctrl.set_vel(1,1)
                    
                    robotxold = robotX
                    
                    [robotX, robotY, robotTheta, unrobotX, unrobotY, unrobotTheta, P] = wall_localization.localize(lidar_data, odometry_data, robotX, robotY, robotTheta,unrobotX, unrobotY, unrobotTheta, P)
                    pred_pos = [robotX, robotY, robotTheta] 
                    currPos = [lidar_data[2], lidar_data[3], lidar_data[4]]
                    
                    print('pred_pos, currPos: ', pred_pos, currPos)



                    
                    act_trajX.append(currPos[0])
                    act_trajY.append(currPos[1])
                    
                    pred_trajX.append(robotX)
                    pred_trajY.append(robotY)
                    
                    unlocalized_trajX.append(unrobotX)
                    unlocalized_trajY.append(unrobotY)
                    
                    plt_robot_pos([robotX, robotY, robotTheta], 'blue')
                    plt_robot_pos([currPos[0], currPos[1],currPos[2]], 'red')
                    plt_robot_pos([unrobotX, unrobotY, unrobotTheta], 'green')
                                
                tick +=1
                    
                plt.xlim(-3, 8)
                plt.ylim(-8,3)
                plt.plot(act_trajX, act_trajY, c = 'red')
                plt.plot(pred_trajX, pred_trajY, c = 'blue')
                plt.plot(unlocalized_trajX, unlocalized_trajY, c = 'green')
                plt.title(f'{tick}')
                plt.pause(0.00001)
                plt.clf()
                
                ax = plt.gca()
                ax.set_aspect('equal', adjustable='box')
                plt.draw()
                                
            ctrl.set_vel(0,0)    
        else:
            print ('Failed to start simulation')
    else:
        print ('Failed connecting to remote API server')
    
    #stop robots
    #initialize_sim.sim_shutdown(client_ID)
    time.sleep(2.0)
    return

#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')