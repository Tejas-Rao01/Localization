# -*- coding: utf-8 -*-
"""
Created on Tue Jan 31 10:37:37 2023
@author: Tejas Rao
"""

from fractions import Fraction
from scipy.odr import * 
import numpy as np 
import cv2
import math 
import matplotlib.pyplot as plt
import time 

# Defining Helper funcitons
class wall_detection():

    def __init__(self, lidar_data, robotX, robotY ,robotTheta, start_angle, end_angle, angle_step):
      
      tick = time.time()
      self.lidar_data = lidar_data
      self.robotX = robotX
      self.robotY = robotY
      self.robotTheta = robotTheta
      
      self.epsilon = 0.1
      self.Min_SeedSeg_len = 5
      
      self.Min_PTS_LS = 6
      self.Delta = 0.5
      self.Num_Pts= lidar_data.shape[0]
      
      self.lidar_angles = np.arange(start_angle, end_angle, angle_step)
     
      self.Gmax = 1
      self.Min_LS_len = 0.4
      self.Min_distp2p = 0.2
      self.sub_sample_lidar(sub_sample_rate=3)      
      
      
 
      args = np.argwhere(np.isinf(self.lidar_data))
      self.lidar_angles = np.delete(self.lidar_angles, args)
      self.lidar_data = np.delete(self.lidar_data, args)
      self.Num_Pts= self.lidar_data.shape[0]

      self.lidar_worldX = np.zeros(self.lidar_data.shape)
      self.lidar_worldY = np.zeros(self.lidar_data.shape)
      
      self.lidar_data2coord()
      self.line_segments = []
      self.break_point_ind = 0
      self.angle_threshold = 5 * math.pi / 180
      self.dist_threshold = 0.0
      tock = time.time()
      print(f'initializing time efficient: {tock - tick}')
    
    def debug(self, num_ticks):
        break_point_ind = self.break_point_ind
        tick = 0 
        indices = []
        while break_point_ind < self.Num_Pts - self.Min_PTS_LS:
            if tick > num_ticks:
                break
            t1 = time.time()
            seed_ind = self.seed_segment_detection(break_point_ind)
            t2 = time.time()
            
            print('time for seed segment detection eff {tick}, {t2-t1}')
            if seed_ind:
                break_point_ind = seed_ind[1] +1
                indices.append(seed_ind)
            else:

                break_point_ind += 1 
                
            tick += 1
        print('num seeds for efficient', len(indices))
    def detected_walls(self):
        
        break_point_ind = self.break_point_ind
        
        # While the break point index is less than total lidar points - min number of points in line segment
        while break_point_ind < self.Num_Pts - self.Min_PTS_LS:
            seed_ind = self.seed_segment_detection(break_point_ind)
            
            # Continue if no seed segment detected 
            if not seed_ind:
                break_point_ind += 1
                continue
                       
            (i, j) = seed_ind
            grown_seed_segment =self.seed_segment_growing((i, j), 0)
                        
            bp_ind = grown_seed_segment[3]
            if bp_ind == ():
                break_point_ind = j + 1
            else:
                break_point_ind = bp_ind

        self.merge_LS_naive()   
            
        return self.line_segments
    
    
    def merge_LS_naive(self):
        
        visited = [False for i in range(len(self.line_segments))]
        merged_line_segments = []
        for i in range(len(self.line_segments)-1):
            if visited[i] == True:
                continue
            p11, p12 = self.line_segments[i]

            
            for j in range(i+1, len(self.line_segments)):

                if visited[j]:
                    continue
                
                visited[i] = True
                
                p21, p22 = self.line_segments[j]

 
                m1, c1 = self.points_2line(p11, p12)
                m2, c2 = self.points_2line(p21, p22)
                
                l1 = self.line_tf_SI2G(m1, c1)
                l2 = self.line_tf_SI2G(m2, c2)
                
                d1 = self.dist_p2l([0,0], l1)
                d2 = self.dist_p2l([0,0], l2)
                
                (y1, x1) = self.projection_point2line([0, 0], m1, c1)
                (y2, x2) = self.projection_point2line([0, 0], m2, c2)
                
                t1 = math.atan2(y1, x1)
                if t1 < 0: 
                    t1 = 2*math.pi + t1
                
                t2 = math.atan2(y2, x2)
                if t2 < 0: 
                    t2 = 2*math.pi + t2
                
                delta_t = min(abs(t1-t2), abs(2 *math.pi - abs(t1-t2)))
                
                r_mean = (d1 + d2)/2
                dist_perc = max(abs(d1-r_mean),abs(d2-r_mean)) / r_mean
                
                
                if delta_t < self.angle_threshold and dist_perc < self.dist_threshold: 
                    
                    visited[j] = True 
                    
                    d1 = self.dist_p2p(p11, p21)
                    d2 = self.dist_p2p(p11, p22)
                    
                    if d1 > d2:
                        p11 = p21
                    else:
                        p12 = p22
                else:                    
                    pass
            merged_line_segments.append([p11, p12])
        
        
        self.line_segments = merged_line_segments
                
    def dist_p2p(self, p1, p2):
    
      x = p1[0] - p2[0]
      y = p1[1] - p2[1]
    
      return np.sqrt(x ** 2 + y ** 2)
       
    def dist_p2l(self, x, y, l):
    
      A, B, C = l

      d = abs(A*x + B*y + C) / (np.sqrt(A**2 + B**2 ))
      
      return d 
    
    # Slope intercept form
    def line_extract2points(self, m, c, p1, p2):
      
      x1 = p1[0]
      y1 = m* x1 + c
      x2 = p2[0]
      y2 = m*x2 + c
   
      return ((x1, y1), (x2, y2))
    
    def line_tf_G2SI(self, A, B, C):
    
      m = -A/B
      c = -C/B
      return m, c
    
    def line_tf_SI2G(self, m, c):

        A, B, C = -m, 1, -c
    
        if A < 0:
            A, B, C = -A, -B, -C
      
        den_a = Fraction(A).limit_denominator(1000).as_integer_ratio()[1]
        den_c = Fraction(C).limit_denominator(1000).as_integer_ratio()[1]
        
        gcd = np.gcd(den_a, den_c)
        lcm = den_a * den_c / gcd
        
        A = A * lcm
        B = B * lcm
        C = C * lcm 

        return A, B, C
  
    def line_intersect_general(self, params1, params2):
    
        a1, b1, c1 = params1
        a2, b2, c2 = params2
        
        # check if lines arent parallel
        
        if a1/a2 == b1/b2 and a1/a2 != c1/c2:
          return None
        
        else:
        
          x = (c1 * b2 - b1 * c2) / (b1 * a2 - a1 * b2)
          y = (a1 *c2 - a2 * c1) / (b1 *a2 - a1 * b2)
        
          return x, y
    
    def points_2line(self, point1, point2):
        x1,y1 = point1    
        x2, y2 = point2       
        if x1 == x2:
          pass  
        else:
          m = (y2- y1)/ (x2 - x1)
          c = y2 -m * x2      
        return m, c
    
    def projection_point2line(self, point, m, c):
        
        x, y = point 
        if m != 0:
          m2 = -1/m 
          c2 = y - m2 * x
        
          intersection_x = - (c - c2) / (m - m2)
          intersection_y = m2 * intersection_x  + c2 
        else:
          intersection_x = x 
          intersection_y = c
        
        return intersection_x, intersection_y
    
    def sub_sample_lidar(self, sub_sample_rate=10):
        
        self.lidar_data = self.lidar_data[np.arange(0,self.Num_Pts-1, sub_sample_rate)]
        self.lidar_angles = self.lidar_angles[np.arange(0,self.Num_Pts-1, sub_sample_rate)]
        self.Num_Pts = len(self.lidar_data)
    
    def lidar_data2coord(self):
        
        self.lidar_worldX = np.multiply(self.lidar_data, np.cos(self.lidar_angles)) + self.robotX
        self.lidar_worldY = np.multiply(self.lidar_data, np.sin(self.lidar_angles)) + self.robotY
       
             
    def linear_func(self, p, x):
    
        m, c = p 
        return m * x + c
    
    def odr_fit(self, x, y):
        
        linear_model = Model(self.linear_func)
        data = RealData(x, y)
        odr_model = ODR(data, linear_model, beta0=[0., 0.])
        out = odr_model.run()
        m, c = out.beta
        return m, c
        
    def predictPoint(self, line_params, sensed_point):
        robotPos = (self.robotX, self.robotY)
        m, c = self.points_2line(robotPos, sensed_point)
        params1 = self.line_tf_SI2G(m, c)
        predx, predy = self.line_intersect_general(params1, line_params)
        return predx, predy 
        
    def seed_segment_detection(self,break_point_ind):

        self.Num_Pts = max(0, self.Num_Pts)
        self.seed_segments = []

        for i in range(break_point_ind, (self.Num_Pts - self.Min_PTS_LS)):
            j  = i + self.Min_SeedSeg_len
            m, c = self.odr_fit(self.lidar_worldX[i:j], self.lidar_worldY[i:j])
  
            line_params = self.line_tf_SI2G(m, c)
            p1, p2 = [], []

            
            
            # Checking if all points are within epsilon distance of the line 
            A,B,C = line_params
            distances_1 = abs(np.add(A*self.lidar_worldX[i:j], B*self.lidar_worldY[i:j]) + C) / (np.sqrt(A**2 + B**2 ))
            if distances_1.max() > self.epsilon:
                continue

            distances_2 = np.sqrt(  np.square(np.subtract(self.lidar_worldX[i:j-1] , self.lidar_worldX[i+1:j])) + np.square(np.subtract(self.lidar_worldY[i:j-1], self.lidar_worldY[i+1:j]))        )

            if distances_2.max() > self.Min_distp2p:
                continue

            return (i, j)
        return None

    def seed_segment_growing(self, indices, break_point):
    
        line_eq = self.line_params
        i, j = indices 
    
        PB, PF = max(break_point, i-1), min(j, len(self.LidarPoints) - 1)
          
        while True:
            if self.dist_p2l(self.LidarPoints[PF][0], line_eq) > self.epsilon:
                PF -= 1
                break

            m, c = self.odr_fit(self.LidarPoints[PB+1:PF])
            line_eq = self.line_tf_SI2G(m, c)
            Point = self.LidarPoints[PF][0]
        
            PF = PF +1 
            if PF > self.Num_Pts -1:
                PF -= 1
                break
            NextPoint = self.LidarPoints[PF][0]
            if self.dist_p2p(Point, NextPoint) > self.Gmax:
                PF -= 1
                break 
       
        while True:
            if self.dist_p2l(self.LidarPoints[PB][0], line_eq) > self.epsilon:
                PB += 1
                break
        
            m, c = self.odr_fit(self.LidarPoints[PB:PF])
            line_eq = self.line_tf_SI2G(m, c)
            Point = self.LidarPoints[PB][0]
            
            PB -= 1
            if PB < 0:
                PB += 1 

                break
            NextPoint = self.LidarPoints[PB][0]
            if self.dist_p2p(Point, NextPoint) > self.Gmax:      
      
   
                break
            
        Len_LS = self.dist_p2p(self.LidarPoints[PB][0],self.LidarPoints[PF][0])
        Num_P = len(self.LidarPoints[PB:PF+1])

        if (Len_LS >= self.Min_LS_len) and (Num_P >= self.Min_PTS_LS):
            
            m, c = self.line_tf_G2SI(line_eq[0], line_eq[1], line_eq[2])
            

            ext_point1 = self.projection_point2line(self.LidarPoints[PB][0], m, c)
            ext_point2 = self.projection_point2line(self.LidarPoints[PF][0], m, c)
            
            if self.dist_p2p(ext_point1, ext_point2) < self.Min_LS_len:
                [(), (), (), (), (), (), ()]
            
            two_points = (ext_point1, ext_point2)
            
            self.line_segments.append(two_points)
            return [self.LidarPoints[PB:PF+1], two_points, (self.LidarPoints[PB+1][0], self.LidarPoints[PF-1][0]), PF, line_eq, (m, c)]
        else:

            return  [(), (), (), (), (), (), ()]
          
          
def wall_2_lidar(wallx, wally, robotX, robotY, robotTheta):
    
    lidar_data = []
    for x, y in zip(wallx, wally):
        
        rel_x = x - robotX
        rel_y = y - robotY
        theta = math.atan2(rel_y, rel_x) - robotTheta +math.pi/2
        r = math.sqrt( rel_y ** 2 + rel_x ** 2)
        
        lidar_data.append([r, theta])
    return lidar_data    
    
def plot_line_segments(points):
    
    for point_pair in points:
        #print(point_pair)
        p1, p2 = point_pair
        #print(('p1, p2'))
        #print(p1)
        plt.plot((p1[0], p2[0]), (p1[1], p2[1]), 'black')
    

if __name__ ==  "__main__":
    
    robotPos = [0, 0, 0]
    robotX, robotY, robotTheta = robotPos 
    