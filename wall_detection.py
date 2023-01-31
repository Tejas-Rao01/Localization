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


# Defining Helper funcitons
class wall_detection():

    def __init__(self, lidar_data, robotX, robotY ,robotTheta):
      self.lidar_data = lidar_data
      self.robotX = robotX
      self.robotY = robotY
      self.robotTheta = robotTheta
      self.LidarPoints = []
      self.epsilon = 0.1
      self.Snum = 6
      self.Pmin = 3
      self.Delta = 10
      self.NP= len(lidar_data)
      self.Gmax = 2
      self.Lmin = 2
      self.Pmin = 7
      self.lidar_data2coord()
      self.line_segments = []
      
    
    def dist_p2p(self, p1, p2):
    
      x = p1[0] - p2[0]
      y = p1[1] - p2[1]
    
      return math.sqrt(x ** 2 + y ** 2)
    
    
    def dist_p2l(self, p, l):
    
      A, B, C = l
      x = p[0]
      y = p[1]
      d = abs(A*x + B*y + C) / (math.sqrt(A**2 + B**2 ))
      
      return d 
    
    # Slope intercept form
    def line_extract2points(self, m, c):
      x1 = 0
      y1 = m* x1 + c
      x2 = 10
      y2 = m*x2 + c
      return ((x1, y1), (x2, y2))
    

    def line_tf_G2SI(self, A, B, C):
    
      m = -A/B
      c = -C/B
      return m, c
    

    def line_tf_SI2G(self, m, c):

        A, B, C = -m, 1, -c
    
        print('A, B, C', A, B, C)
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
    
    
    
    def lidar_data2coord(self):
        
        for point in self.lidar_data:
       
         r, theta = point # theta starts from zero wrt robot
       
         angle = theta - math.pi / 2 + self.robotTheta 
         x_coord = self.robotX + r * math.cos(angle)
         y_coord = self.robotY + r * math.sin(angle)
       
         coord = [x_coord, y_coord]
         self.LidarPoints.append([coord, angle])
          
      # Define a linear function to fit our data with 
    
    def linear_func(self, p, x):
    
        m, c = p 
        return m * x + c
    
    def odr_fit(self, laser_points):
        
        x = np.array([i[0][0] for i in laser_points])
        y = np.array([i[0][1] for i in laser_points])
        
        linear_model = Model(self.linear_func)
        
        data = RealData(x, y)
        odr_model = ODR(data, linear_model, beta0=[0., 0.])
        
        out=  odr_model.run()
        m, c = out.beta
        
        return m, c
        
    def predictPoint(self, line_params, sensed_point):
        robotPos = (self.robotX, self.robotY)
        m, c = self.points_2line(robotPos, sensed_point)
        params1 = self.line_tf_SI2G(m, c)
        predx, predy = self.line_intersect_general(params1, line_params)
        return predx, predy 
        
    
    
    
    def seed_segment_detection(self,break_point_ind):

        Flag = True 
        self.NP = max(0, self.NP)
        self.seed_segments = []
        
# =============================================================================
#         print('cp 1 ')
# =============================================================================
        
        for i in range(break_point_ind, (self.NP - self.Pmin)):
        
          j = i + self.Snum
        # =============================================================================
        #       print('j = ' , j)
        #       print('self.lidar_pionts ', self.LidarPoints)
        # =============================================================================
          m, c = self.odr_fit(self.LidarPoints[i:j])
          
        # =============================================================================
        #       two_points = self.line_extract2points(m, c)
        #       plt.figure()
        #       plt.plot((two_points[0][0],two_points[1][0]) , (two_points[0][1],two_points[1][1] ))
        #       plt.xlim(-7,7)
        #       plt.ylim(-7,7)
        # =============================================================================
# =============================================================================
#           print('m, c = ', m, c)
#           print('cp -2')
# =============================================================================
          
          line_params= self.line_tf_SI2G(m, c)
# =============================================================================
#           print('line params: ' , line_params)
# =============================================================================
          
          for k in range(i, j):
            # condition-1, dist to line < epistl 
            
            #print(' cp 3 ')
            print(self.LidarPoints[k][0], line_params)
            d1 = self.dist_p2l(self.LidarPoints[k][0], line_params)
        
            if d1 > self.epsilon:
              Flag = False
              break
            
            predicted_point = self.predictPoint(line_params, self.LidarPoints[k][0])
            d2 = self.dist_p2p(predicted_point, self.LidarPoints[k][0])
        
            if d2 > self.Delta:
              Flag = False
              break
          if Flag == True:
            self.line_params = line_params 
            return self.LidarPoints[i:j]

     
    
    def seed_segment_growing(self, indices, break_point):
    
        line_eq = self.line_params
      
        i, j = indices 
      
        # Beginnning and Final Points of the line segments 
        
        PB, PF = max(break_point, i-1), min(j+1, len(self.LidarPoints) - 1)
        
        print('cp 4')
        print('self np ', self.NP)
        # Extending on the right side 
        
        while True:
            if self.dist_p2l(self.LidarPoints[PF][0], line_eq) > self.epsilon:
                PF -= 1
                break
                
            print('PF: ', PF)
            print('cp 5')
            
            m, c = self.odr_fit(self.LidarPoints[PB:PF])
            line_eq = self.line_tf_SI2G(m, c)
            Point = self.LidarPoints[PF][0]
        
            PF = PF +1 
            if PF > self.NP -1:
                PF -= 1
                break
            NextPoint = self.LidarPoints[PF][0]
            
            
            if self.dist_p2p(Point, NextPoint) > self.Gmax:
                print('cpp 8')
                break 

     
      
        print('cp 10')

        while self.dist_p2l(self.LidarPoints[PB][0], line_eq) < self.epsilon:
            if PB < 1:
                 print('PB ', PB)
                 print('cp 11')
                 break
            else:
                m, c = self.odr_fit(self.LidarPoints[PB:PF])
                line_eq = self.line_tf_SI2G(m, c)
                Point = self.LidarPoints[PB][0]
            PB -= 1
            NextPoint = self.LidarPoints[PB][0]
            if self.dist_p2p(Point, NextPoint) > self.Gmax:
                break
            
        Len_LS = self.dist_p2p(self.LidarPoints[PB][0],self.LidarPoints[PF][0])
        Num_P = len(self.LidarPoints[PB:PF+1])
        print('Len_LS  ', Len_LS)
        print('NuM P ', Num_P)
        print('PB', PB)
        print('Pf ', PF)
        if (Len_LS >= self.Lmin) and (Num_P >= self.Pmin):
            self.line_params = line_eq
            m, c = self.line_tf_G2SI(line_eq[0], line_eq[1], line_eq[2])
            self.two_points = self.line_extract2points(m, c)
            self.line_segments.append([self.LidarPoints[PB+1][0], self.LidarPoints[PF-1][0]])
            #print(self.LidarPoints[PB:PF])
            return [self.LidarPoints[PB:PF+1], self.two_points, (self.LidarPoints[PB+1][0], self.LidarPoints[PF-1][0]), PF, line_eq, (m, c)]
        else:
            return False 
          
          
def wall_2_lidar(wallx, wally, robotX, robotY, robotTheta):
    
    lidar_data = []
    for x, y in zip(wallx, wally):
        
        rel_x = x - robotX
        rel_y = y - robotY
        theta = math.atan2(rel_y, rel_x) - robotTheta +math.pi/2
        r = math.sqrt( rel_y ** 2 + rel_x ** 2)
        
        lidar_data.append([r, theta])
    return lidar_data    
    
    
if __name__ ==  "__main__":
    
    robotPos = [0, 0, 0]
    robotX, robotY, robotTheta = robotPos 
    num_points = 10
    wallx = np.zeros([1, num_points])+ 1 + np.random.normal(0,0.01,num_points) + 1
    wallx = wallx.squeeze()
    #wally[-1] = 3
    wally = np.linspace(-5, 5, num_points)
    
    plt.xlim((-7,7))
    plt.ylim((-7, 7))
    plt.scatter(wallx, wally)
    
    
    #  
    lidar_data  = wall_2_lidar(wallx, wally, robotX, robotY, robotTheta)
    wall_detector = wall_detection(lidar_data, robotX, robotY, robotTheta) 
    seed_segment = wall_detector.seed_segment_detection(0)
    grown_seed_segment = wall_detector.seed_segment_growing((0, 6), 0)
    
        

    print(wallx)
    print(grown_seed_segment[0])

    
    
    
    