# -*- coding: utf-8 -*-
"""
Created on Tue Dec 13 11:26:17 2022

@author: Tejas Rao
"""

import sim

class Localize():
    
    
    
    def __init__(self, client_ID, channel):
        
        
        self.client_ID = client_ID
        self.channel = channel


    def get_signal(self):
        
        res, self.signal = sim.simxGetStringSignal(self.client_ID, self.channel, sim.simx_opmode_streaming)
        self.data = sim.simxUnpackFloats(self.signal)
        if self.data:
           # print(self.data)
            print("packet recieved, length: ", len(self.data))
            
        return (self.process_signal())
        
    def process_signal(self):
        
        signal = self.data
        if len(signal) > 0:
            
            self.robot_X = signal[0]
            self.robot_Y = signal[1]
            self.robot_theta = signal[2]
            self.num_points = signal[3]
            self.angle_range = signal[4]
            self.start_angle = signal[5]
            self.step_size = signal[6]
            self.l_ticksX = signal[7]
            self.l_ticksY = signal[8]
            self.r_ticksX = signal[9]
            self.r_ticksY = signal[10]
            
            
            
            #Process the signal
            self.lidar_data = []
            angle = self.start_angle
            for i in range(11, len(signal)):
                self.lidar_data.append([angle, signal[i]])
                angle += self.step_size
            
            # Predicted Pos 
            file = open('motor_ticks.txt', 'a')
            content = ' '.join(map(str,[self.l_ticksX, self.l_ticksY, self.r_ticksX, self.r_ticksY])) + '\n'
            file.write(content)        
            file.close()
                        
        
            currPos = [self.robot_X,self.robot_Y,self.robot_theta]
            file = open('actual_pos.txt', 'a')
            content = ' '.join(map(str,currPos)) + '\n'
            file.write(content)        
            file.close()
            
            f = open('scan.txt', 'a')
            for i in range(len(self.lidar_data)):
                f.write(f'{self.lidar_data[i][0]} {self.lidar_data[i][1]} ')
                #print(f'{self.lidar_data[i][0]} {self.lidar_data[i][1]}')
            f.write('\n')
            f.close()
            print('contents written ', len(self.lidar_data))
            return True 
        return False
            

    
            
            
        
        
        
        