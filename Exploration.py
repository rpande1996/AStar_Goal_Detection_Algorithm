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

    def obstacles_form(self, image):
        """
        Create all obstacles in the images by calling various obstacle functions

        Parameters
        ----------
        image : np.array
            InputsImage for adding obstacle
        """
        major_axis = 60
        minor_axis = 30
        c_y = 246
        c_x = 145
        c_y1 = 90
        c_x1 = 70
        radius = 35
        for i in range(len(image)):
            for j in range(len(image[0])):
                self.ellipse(image, major_axis, minor_axis, i, j, c_x, c_y)
                self.circle(image, radius, i, j, c_x1, c_y1)
                self.slanted_rect(image, i, j)
                self.polygon(image, i, j)
                self.c_shape(image, i, j)
        # exploration.c_shape(image,i,j)

    def goal_reached(self):
        """
        Checks if the goal is reached or not if reached return True
        and if not reached continues exploring

        Parameters
        ----------

        Returnss
        -------
        Boolean : bool
            True or False depending on the current state reached the goal or not

        """
        pos_0 = self.goal[0]
        pos_1 = self.goal[1]
        # self.start_score=self.string(self.start[0],self.start[1])
        # self.data_with_string[self.start_score]=self.start
        # self.goal_score=self.string(pos_0,pos_1)
        if self.h(self.current_score[0], self.current_score[1]) <= 1.5 * 5:
            self.goal_score = self.string(self.current_score[0], self.current_score[1], self.current_score[2])
            print("goal_reached")
            # print(len(self.expanded))
            # print("self.expanded",self.expanded)
            return True
        return False

    def string(self, pos_0, pos_1, n):
        """
        Converts the list of the state into string for easy comparison
        when further converted into integer

        Parameters
        ----------
        pos_0 : Int
            x-coordinate of current state
        pos_0 : Int
            y-coordinate of current state

        Returns
        -------
        c : str
            String of the state

        """
        n = int(n)
        if pos_0 < 10:
            pos_0 = "00" + str(pos_0)
        elif pos_0 < 100:
            pos_0 = "0" + str(pos_0)

        if n < 10:
            n = "0" + str((n))

        if pos_1 < 10:
            pos_1 = "00" + str(pos_1)
        elif pos_1 < 100:
            pos_1 = "0" + str(pos_1)

        # pos
        c = ""

        c = str(pos_0) + str(pos_1) + str(n)
        # print("c",c)
        return c

    def h(self, pos_0, pos_1):
        # cost=abs(pos_0-self.goal[0])+abs(pos_1-self.goal[1])
        # print(self.goal)
        cost = ((pos_0 - self.goal[0]) ** 2 + (pos_1 - self.goal[1]) ** 2) ** (1 / 2)
        # print(pos_0,pos_1,cost)
        return cost

    def action_space(self, image, pos_0, pos_1, theta_d):
        d = self.step_size
        theta_d = theta_d * self.theta_diff
        parent = self.string(pos_0, pos_1, round(theta_d / self.theta_diff))
        self.parent_pos = (pos_1, 299 - pos_0)
        # print("self.parent_pos",self.parent_pos)
        # print("parent",parent,theta_d)
        for theta in range(-60, 60 + self.theta_diff, self.theta_diff):
            # print(1)
            # print(pos_0,pos_1)
            if (pos_0 >= 0 + self.padding and pos_1 >= 0 + self.padding) and (
                    pos_0 <= 299 + self.padding and pos_1 <= 399 - +self.padding):
                # print(self.cost_togo)
                # print(2)
                theta_net = theta_d + theta
                if theta_net >= 360:
                    theta_net = theta_net - 360
                if theta_net < 0:
                    theta_net = 360 + theta_net
                x = round(d * np.sin(np.deg2rad(theta_net)) + pos_0)
                y = round(d * np.cos(np.deg2rad(theta_net)) + pos_1)
                n = theta_net / self.theta_diff
                if (x >= 0 + self.padding and y >= 0 + self.padding) and (
                        x <= 299 - self.padding and y <= 399 - self.padding):
                    # print(3)
                    score = self.string(x, y, n)
                    # print("updated value",x,y)
                    # print("self.cost_togo[parent,round(theta_d/self.theta_diff)]",self.cost_togo[parent,round(theta_d/self.theta_diff)])
                    # if  np.array_equiv(image[299-pos_0,pos_1,:],np.array([00,00,0])):
                    #    continue
                    # else:
                    if np.array_equiv(image[299 - x, y, :], np.array([00, 00, 0])) or self.image_p[
                        round(x / self.threshold), round(y / self.threshold), int(n)] == 2:
                        pass
                    elif self.image_p[round(x / self.threshold), round(y / self.threshold), int(n)] == 1:
                        # print(self.cost_togo[parent,round(theta_d/self.theta_diff)])
                        # print(self.cost_togo)
                        # print(self.cost[score,round(theta_net/self.theta_diff)])
                        if self.cost[score, round(theta_net / self.theta_diff)] > self.cost_togo[
                            parent, round(theta_d / self.theta_diff)] + 1 + self.h(x, y):
                            self.cost_togo[score, round(theta_net / self.theta_diff)] = self.cost_togo[parent, round(
                                theta_d / self.theta_diff)] + 1
                            self.cost[score, round(theta_net / self.theta_diff)] = self.cost_togo[score, round(
                                theta_net / self.theta_diff)] + self.h(x, y)
                            self.frontier[x, y, round(theta_net / self.theta_diff)] = self.cost[
                                score, round(theta_net / self.theta_diff)]
                            self.parent_orignal_data[score] = parent
                    else:
                        # print(self.cost_togo)
                        # print(self.cost_togo[parent,round(theta_d/self.theta_diff)]+1)
                        self.cost_togo[score, round(theta_net / self.theta_diff)] = self.cost_togo[parent, round(
                            theta_d / self.theta_diff)] + 1
                        # print( self.cost_togo[score,round(theta_net/self.theta_diff)])
                        # print("x,y",x,y,theta_net,self.cost_togo[score,round(theta_net/self.theta_diff)])
                        self.cost[score, round(theta_net / self.theta_diff)] = self.cost_togo[score, round(
                            theta_net / self.theta_diff)] + self.h(x, y)
                        # self.cost[score,round(theta_net/self.theta_diff)]=self.cost_togo[score,round(theta_net/self.theta_diff)]+self.h(pos_0,pos_1)
                        # print("self.cost[score,round(theta_net/self.theta_diff)]",self.cost[score,round(theta_net/self.theta_diff)],score)
                        # self.frontier[pos_0,pos_1,round(theta_net/self.theta_diff)]=self.cost[score,round(theta_d/self.theta_diff)]
                        self.parent_orignal_data[score] = parent
                        # if self.image_p[round(pos_0/self.threshold),round(pos_1/self.threshold),round(theta_net/self.theta_diff)]==0: #or np.array_equiv(image[299-pos_0,pos_1,:],np.array([00,00,0])):
                        # if self.image_p[round(pos_0/self.threshold),round(pos_1/self.threshold),round(theta_net/self.theta_diff)]==0:

                        self.frontier[x, y, round(theta_net / self.theta_diff)] = self.cost[
                            score, round(theta_net / self.theta_diff)]
                        self.data_with_string[score] = [pos_0, pos_1]
            # frontier_list[]
            # print(x,y,theta_d+theta)
            # print("self.cost",self.cost)
        return image, x, y

    def expanding(self, pos_0, pos_1, n):
        """
            This function checks if the node is in expanded /visited list and
            if it not then appends in the expanded list


            Parameters
            ----------

            pos_0 : Int
                x_coordinate of the current node
            pos_1 : Int
                y_coordinate of the current node

            Returns
            -------

        """
        cnvt_front = self.string(pos_0, pos_1, n)
        if int(cnvt_front) in self.expanded:

            a = 1
        else:
            self.expanded.append(int(cnvt_front))

    def frontier_list(self, image):
        """
            This function checks if the node is in expanded/visited list  and
            pops out untill it finds a node that has not been visited/expanded.


            Parameters
            ----------

            Returns
            -------

            pos_0 : Int
                x_coordinate of the current node
            pos_1 : Int
                y_coordinate of the current node



        """
        # p=min(self.frontier)
        # pos_0,pos_1=self.frontier[p]
        # print(min(self.frontier,key=self.frontier.get),self.frontier[min(self.frontier,key=self.frontier.get)])
        # print(self.frontier)
        pos_0, pos_1, n = min(self.frontier, key=self.frontier.get)
        # print("value from frontier",pos_0,pos_1,n)
        n = int(n)
        theta = n * self.theta_diff
        # self.frontier=dict(sorted(self.frontier.items(),key=lambda x: x[1],reverse=True))
        # print("frontier",type(self.frontier))
        # pos_0,pos_1=min(self.frontier,key=self.frontier.get)
        # pos_0,pos_1=self.frontier,key=self.frontier.get)
        # [pos_0,pos_1],cost=self.frontier.popitem()
        # pos_0,pos_1=self.frontier.pop(0)
        self.current_score = [pos_0, pos_1, n]

        cost = self.frontier[pos_0, pos_1, n]
        # print(pos_0,pos_1,theta,cost)
        # print(self.frontier)
        del self.frontier[pos_0, pos_1, n]

        # print(self.frontier)
        # print("self.image_p[pos_0,pos_1,n]",self.image_p[pos_0,pos_1,n])
        # print(2)
        if self.image_p[round(pos_0 / self.threshold), round(pos_1 / self.threshold), n] == 1 or np.array_equiv(
                image[299 - pos_0, pos_1, :], np.array([00, 00, 0])) or self.image_p[
            round(pos_0 / self.threshold), round(pos_1 / self.threshold), n] == 2:

            # print(1)
            # elif int(self.current_score) in self.obstacle:
            # pos_0,pos_1,theta_d=self.frontier_list(image)
            return self.frontier_list(image)
        # else:
        # print(pos_0,pos_1,n*self.theta_diff,cost)
        # print(3)
        else:
            image[299 - pos_0, pos_1, :] = 200, 200, 0
            self.image_p[
                round(pos_0 / self.threshold), round(pos_1 / self.threshold), round(theta / self.theta_diff)] = 1
            """if int(self.current_score) in self.expanded:

                 self.frontier_list(image)
            elif int(self.current_score) in self.obstacle:
                 self.frontier_list(image)"""
            # print("cost", pos_0,pos_1,cost)
            # cv2.line(image,self.parent_pos,(pos_1,299-pos_0),(200,200,0),1)
            image = image.astype(np.uint8)
        # print("2nd_check",pos_0,pos_1,n*self.theta_diff,cost)
        # print("expanded",self.expanded)
        # print(pos_0,pos_1,n)
        return pos_0, pos_1, n

        # image=image.astype(np.uint8)
        # print("2nd_check",pos_0,pos_1,n*self.theta_diff,cost)
        # print("expanded",self.expanded)
        # return pos_0,pos_1,n*self.theta_diff

    def circle(self, image, radius, i, j, c_x, c_y):
        """
            This function give points that lies in circle.


            Parameters
            ----------
            image : np.array
                This is the image or state in which the location of obstacle is updated
            radius : Int
                Radius of the circle
            i : Int
                x_coordinate of point
            j : Int
                y coordinate of points
            c_x:Int
                x coordinate of center of the circle
            c_y:Int
                y coordinate of center of the circle
            Returns
            -------





        """
        major_axis = radius
        minor_axis = radius
        self.ellipse(image, major_axis, minor_axis, i, j, c_x, c_y)

    def ellipse(self, image, major_axis, minor_axis, i, j, c_x, c_y):
        """
            This function give points that lies in ellipse.


            Parameters
            ----------
            image : np.array
                This is the image or state in which the location of obstacle is updated
            radius : Int
                Radius of the circle
            major_axis: Int
                elongation of elipse across y axis
            minor_axis: Int
                elogation of ellipse across x axis
            i : Int
                x_coordinate of point
            j : Int
                y coordinate of points
            c_x:Int
                x coordinate of center of the circle
            c_y: Int
                y coordinate of center of the circle

            Returns
            -------





        """
        if (((i - c_x) / (minor_axis + self.padding)) ** 2 + ((j - c_y) / (major_axis + self.padding)) ** 2) <= 1:
            # print("yes")
            image[299 - i, j, :] = 0, 0, 0
            self.image_p[i, j, :] = 2
            # self.obstacle.append([i,j])

    def slanted_rect(self, image, i, j):
        """
            This function give points that lies in slanted rectangle.


            Parameters
            ----------
            image : np.array
                This is the image or state in which the location of obstacle is updated

            i : Int
                x_coordinate of point
            j : Int
                y coordinate of points


            Returns
            -------

        """
        s1 = 0.7
        s2 = -1.42814
        x1 = np.arctan(s1)
        x2 = np.arctan(s2)
        d1 = np.cos(np.pi - x1)
        d2 = np.cos(np.pi - x2)
        a = -(self.padding / d1)
        b = -(self.padding / d2)
        if (-0.7 * j + 1 * i) >= (73.4 - a) and (i + 1.42814 * j) >= (172.55 - b) and (-0.7 * j + 1 * i) <= (
                99.81 + a) and (i + 1.42814 * j) <= (429.07 + b):
            image[299 - i, j, :] = 0, 0, 0
            self.image_p[i, j, :] = 2

    def c_shape(self, image, i, j):
        """
            This function give points that lies in c shape.


            Parameters
            ----------
            image : np.array
                This is the image or state in which the location of obstacle is updated

            i : Int
                x_coordinate of point
            j : Int
                y coordinate of points


            Returns
            -------





        """

        if ((i <= 280 + self.padding) and (j >= 200 - self.padding) and (i >= 230 - self.padding) and (
                j <= 230 + self.padding)) and not (
                (i <= 270 - self.padding) and (j >= 210 + self.padding) and (i >= 240 + self.padding) and (
                j <= 230 + self.padding)):
            image[299 - i, j, :] = 0, 0, 0
            self.image_p[i, j, :] = 2

    def polygon(self, image, i, j):
        """
            This function give points that lies in complex polygon.


            Parameters
            ----------
            image : np.array
                This is the image or state in which the location of obstacle is updated

            i : Int
                x_coordinate of point
            j : Int
                y coordinate of points


            Returns
            -------





        """
        if (
                i + j >= 391 and j - i <= 265 and i + 0.81 * j <= 425.66 and i + 0.17 * j <= 200 and 0.89 * j - i >= 148.75) or (
                13.5 * j + i <= 5256.72 and 1.43 * j - i >= 368.82 and i + 0.81 * j >= 425.66):
            image[299 - i, j, :] = 0, 0, 0
            self.image_p[i, j, :] = 2
            # print(2)

    def backtracking(self, image, out, image_list):
        """
            The function backtracks from goal to start node
            and gives an path


            Parameters
            ----------
            image : np.array
                This is the image or state in which the location of obstacle is updated
            out : np.array
                video writing array
            image_list:
                list of frames that have been saved  while exploring

            Returns
            -------
            path : list
                Returns the path that needs to be followed
            image_list : List
                Returns list of images/ frames used in exploration and backtracking





        """
        loop = self.parent_orignal_data[self.goal_score]
        # print("self.goal_score",self.goal_score)

        path = [self.goal]
        # print(self.start_score)
        while int(self.start_score) != int(loop):
            #    pass
            # loop
            parent_pos = self.data_with_string[loop]
            # print("loop",loop)
            # print(loop)
            # print("start",self.start_score)
            loop = self.parent_orignal_data[loop]
            index = self.data_with_string[loop]
            # print(index)
            if index == self.start:
                break
            cv2.line(image, (parent_pos[1], 299 - parent_pos[0]), (index[1], 299 - index[0]), (255, 0, 0), 1)
            path.append(index)
            image[299 - index[0], index[1], :] = 255, 0, 0
            out.write(image)
            image_list.append(image)

        return path, image_list