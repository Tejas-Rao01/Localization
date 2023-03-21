#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 21 13:50:00 2023

@author: shreyash
"""
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import rospy
import message_filters
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import wall_localization
import matplotlib.pyplot as plt
import numpy as np
import std_msgs.msg


class Node(object):
    def __init__(self):
        
        # init node 
        rospy.init_node("localizer", anonymous=True)
        
        # Params
        self.robotPos = [-1,0,0]
        self.robotX = self.robotPos[0]
        self.robotY = self.robotPos[1]
        self.robotTheta = self.robotPos[2]
        self.rate = rospy.Rate(10)
        self.plot_vars = []
        
        self.unrobotX, self.unrobotY, self.unrobotTheta = self.robotPos
        
        self.SL, self.SR = 0,0
        self.P = np.eye(3)
        
        self.step_size = np.pi/1147 * 2
        self.odometry_data = [0,self.SL, self.SR]
        self.fig = plt.figure()

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(10)

        # Subscribers
        lidar_sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        # odom_sub =  rospy.Subscriber("/odom", std_msgs.msg.Float64, self.callback)
        
        
        
    def callback(self, lidar_scan):
        self.lidar_data = lidar_scan.ranges
        [self.robotX, self.robotY, self.robotTheta, self.unrobotX, self.unrobotY, self.unrobotTheta, self.P, self.plot_vars] = wall_localization.localize(self.lidar_data, self.step_size, self.odometry_data, self.robotX, self.robotY, self.robotTheta, self.unrobotX, self.unrobotY, self.unrobotTheta, self.P)
        self.robotPos = [self.robotX, self.robotY, self.robotTheta]
        print('Localized robot')
        print(f'robotPos: {self.robotPos}')
    
    def plot(self):
        if self.plot_vars !=[]:
            print(self.plot_vars)        
            X, Y, robotX, robotY, robotTheta, walls = self.plot_vars
            transformation_mat = np.array([ [np.cos(robotTheta), -np.sin(robotTheta), robotX],[np.sin(robotTheta), np.cos(robotTheta), robotY],[0,0,1]])
            for wall in walls:
                p1, p2 = wall
                p1 = list(p1)
                p2 = list(p2)
                p1.append(1)
                p2.append(1)
                p1 = np.matmul(transformation_mat, np.array(p1).reshape((3,1)))
                p2 = np.matmul(transformation_mat, np.array(p2).reshape( (3,1)))
                
                x = [p1[0], p2[0]]
                y = [p1[1], p2[1]]
                plt.plot(x,y, c='black')
                plt.scatter(X, Y, c = 'red', linewidths=0.1)
                plt.show()
                plt.pause(0.1)
                plt.clf()
        
    def main(self):
        
        while not rospy.is_shutdown():
            self.plot()
            self.rate.sleep()



if __name__ == '__main__':    
    node = Node()
    node.main()







