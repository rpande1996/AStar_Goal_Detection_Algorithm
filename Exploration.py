#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Mar  4 12:56:40 2021

@author: abhi
"""
import numpy as np
import cv2


class exploration_r:
    def __init__(self, start, goal, step_size, theta_s, theta_g):
        """
        Intializes variables for the class. Stores the goal state and
        start state


        Parameters
        ----------
        start : list
            Starting point coordinates
        goal : list
            Goal point coordinates

        """
        clearance = 5
        radius = 10
        self.padding = clearance + radius
        self.ground_truth = {}

        self.obstacle = []

        self.expanded = []

        self.parent = []
        self.parent_orignal_data = {}

        self.start = start
        # print(self.start)
        self.theta = theta_s
        self.theta_diff = 30
        self.n = int(self.theta / self.theta_diff)
        self.frontier = {}
        self.frontier[self.start[0], self.start[1], self.n] = 0
        self.start_score = self.string(self.start[0], self.start[1], self.n)
        self.frontier_string = []
        self.cost_togo = {}
        self.cost_togo[self.start_score, self.n] = 0
        self.cost = {}
        # self.cost=0
        self.goal = goal
        self.cost[self.start_score, self.n] = self.cost_togo[self.start_score, self.n] + self.h(self.start[0],
                                                                                                self.start[1])
        self.data_with_string = {}

        self.current_score = "00"

        self.theta_diff = 30
        # self.cost={}
        # self.cost[self.start_score]=0
        self.step_size = step_size
        self.threshold = 1
        self.parent_pos = (self.start[1], 299 - self.start[0])

        self.image_p = np.zeros(
            [round(300 / self.threshold), round(400 / self.threshold), round(360 / self.theta_diff)])

    def obstacle_prone_area(self, image):
        """
        Checks if the goal state or start state is in the obstacle area

        Parameters:
        -----------
        image : np.array
            Inputs image for adding obstacle

        Returns
        -------
        Boolean : Boolean
            Returns True if wither of goal or start is in obstacle space
            else returns False


        """

        start_x = self.start[0]
        start_y = self.start[1]
        goal_x = self.goal[0]
        goal_y = self.goal[1]

        if (np.array_equiv(image[299 - goal_x, goal_y, :], np.array([0, 0, 0]))) or (
                np.array_equiv(image[299 - start_x, start_y, :], np.array([0, 0, 0]))):
            # print(1)
            return False
        else:
            # print(2)
            return True


    def obstacles_form(self,image):
        """
        Create all obstacles in the images by calling various obstacle functions

        Parameters
        ----------
        image : np.array
            InputsImage for adding obstacle
        """
        major_axis=60
        minor_axis=30
        c_y=246
        c_x=145
        c_y1=90
        c_x1=70
        radius=35
        for i in range(len(image)):
            for  j in range(len(image[0])):

                self.ellipse(image,major_axis,minor_axis,i,j,c_x,c_y)
                self.circle(image,radius,i,j,c_x1,c_y1)
                self.slanted_rect(image,i,j)
                self.polygon(image,i,j)
                self.c_shape(image,i,j)
        #exploration.c_shape(image,i,j)