# -*- coding: utf-8 -*-
"""
Created on Mon Dec 12 22:33:55 2022

@author: Tejas Rao
"""



import numpy as np 
import matplotlib.pyplot as plt


file = open('actual_pos.txt')
f = file.readlines()

x_coords = []
y_coords = []
for line in f:
    coords = list(map(float,line.split()))
    x_coords.append(coords[0])
    y_coords.append(coords[1])
    

plt.scatter(x_coords, y_coords)
    
