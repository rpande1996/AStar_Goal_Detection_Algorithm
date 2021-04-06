#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar  3 19:15:58 2021

@author: abhi
"""

from Exploration import exploration_r
import numpy as np
import cv2

image=255*np.ones((300,400,3))
#print(image)

parent_orignal_data=[]

#Inputting coordinates

print(" Start coordinates ")
s_y=input(" x_coordinate : ")
s_x=input(" y_coordinate : ")
theta_s=int((input(" theta_start : ")))
print(" Goal coordinates ")
g_y=input(" x_coordinate : ")
g_x=input(" y_coordinate : ")
theta_g=int(input(" theta_goal : "))
print(" Other parameters ")
step=int(input(" step size : "))

#print(s_x,s_y,g_x,g_y)0
start=[int(s_x),int(s_y)]
goal=[int(g_x),int(g_y)]
i=0
image=image.astype(np.uint8)

# object of class
exploration=exploration_r(start,goal,step,theta_s,theta_g)

exploration.obstacles_form(image)