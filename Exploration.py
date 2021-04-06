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