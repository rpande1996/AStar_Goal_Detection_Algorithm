import cv2
import numpy as np


class exploration_r:
    def __init__(self, start, goal, step_size, theta_s, theta_g, clearance, radius):
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
        self.padding = clearance + radius
        self.ground_truth = {}

        self.obstacle = []

        self.expanded = []

        self.parent = []
        self.parent_orignal_data = {}

        self.start = start
        self.theta = theta_s
        self.theta_diff = 30
        self.n = int(self.theta / self.theta_diff)
        self.frontier = {}
        self.frontier[self.start[0], self.start[1], self.n] = 0
        self.start_score = self.string(self.start[0], self.start[1], self.n)
        self.frontier_string = []
        self.cost_togo = {}
        self.cost_togo[self.start_score, self.n] = 0
        self.parent_orignal_data[self.start_score] = None
        self.cost = {}
        # self.cost=0
        self.goal = goal
        self.cost[self.start_score, self.n] = self.cost_togo[self.start_score, self.n] + self.h(self.start[0],
                                                                                                self.start[1])
        self.data_with_string = {}
        self.data_with_string[self.start_score] = self.start
        self.current_score = "00"
        self.i = 1
        self.theta_diff = 30
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
            return False
        else:
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
                self.boundary(image, i, j)
                self.c_shape(image, i, j)

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
        if self.h(self.current_score[0], self.current_score[1]) <= self.step_size:
            self.goal_score = self.string(self.current_score[0], self.current_score[1], self.current_score[2])
            print("goal_reached")
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
        c = str(pos_0) + str(pos_1) + str(n)
        return c

    def h(self, pos_0, pos_1):

        """
            This function returns heuristic value/the eucledian cost of the node


            Parameters
            ----------

            pos_0 : Int
                x_coordinate of the current node
            pos_1 : Int
                y_coordinate of the current node

            Returns
            -------
            cost: Float
                heutistic cost/Eucledian cost of the node 
            

        """

        cost = ((pos_0 - self.goal[0]) ** 2 + (pos_1 - self.goal[1]) ** 2) ** (1 / 2)

        return cost

    def action_space(self, image, pos_0, pos_1, theta_d):

        """
            This function performs the action and saves the nodes, parents and children
            and nodes to be visited. It calculates the cost and updates the costs.
            


            Parameters
            ----------
            image: Array
                Image for displaying
            pos_0 : Int
                x_coordinate of the current node
            pos_1 : Int
                y_coordinate of the current node
            theta_d: Int
                Theta coordinate of the current node
            Returns
            -------
            image: Array
                Image for displaying
            x : Int
                x_coordinate of the node
            y : Int
                y_coordinate of the node

            

        """

        d = self.step_size
        theta_d = theta_d * self.theta_diff
        parent = self.string(pos_0, pos_1, round(theta_d / self.theta_diff))
        self.parent_pos = (pos_0, pos_1)

        for theta in range(-60, 60 + self.theta_diff, self.theta_diff):
            # print(1)

            if (pos_0 >= 0 + self.padding and pos_1 >= 0 + self.padding) and (
                    pos_0 <= 299 + self.padding and pos_1 <= 399 - +self.padding):

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

                    score = self.string(x, y, n)

                    if np.array_equiv(image[299 - x, y, :], np.array([00, 00, 0])) or self.image_p[
                        round(x / self.threshold), round(y / self.threshold), int(n)] == 2:
                        pass
                    elif self.image_p[round(x / self.threshold), round(y / self.threshold), int(n)] == 1:

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

                        self.cost_togo[score, round(theta_net / self.theta_diff)] = self.cost_togo[parent, round(
                            theta_d / self.theta_diff)] + 1

                        self.cost[score, round(theta_net / self.theta_diff)] = self.cost_togo[score, round(
                            theta_net / self.theta_diff)] + self.h(x, y)

                        self.parent_orignal_data[score] = parent

                        self.frontier[x, y, round(theta_net / self.theta_diff)] = self.cost[
                            score, round(theta_net / self.theta_diff)]
                        self.data_with_string[score] = [pos_0, pos_1]

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

        pos_0, pos_1, n = min(self.frontier, key=self.frontier.get)

        n = int(n)
        theta = n * self.theta_diff

        self.current_score = [pos_0, pos_1, n]

        cost = self.frontier[pos_0, pos_1, n]

        del self.frontier[pos_0, pos_1, n]

        if self.image_p[round(pos_0 / self.threshold), round(pos_1 / self.threshold), n] == 1 or np.array_equiv(
                image[299 - pos_0, pos_1, :], np.array([00, 00, 0])) or self.image_p[
            round(pos_0 / self.threshold), round(pos_1 / self.threshold), n] == 2:

            return self.frontier_list(image)

        else:
            score = self.string(pos_0, pos_1, n)

            if self.step_size == 1:
                image[299 - pos_0, pos_1, :] = 200, 200, 0
                self.image_p[
                    round(pos_0 / self.threshold), round(pos_1 / self.threshold), round(theta / self.theta_diff)] = 1
            elif self.parent_orignal_data[score] is not None:
                parent = self.parent_orignal_data[score]
                self.parent_pos = self.data_with_string[parent]
                self.image_p[
                    round(pos_0 / self.threshold), round(pos_1 / self.threshold), round(theta / self.theta_diff)] = 1
                cv2.line(image, (pos_1, 299 - pos_0), (self.parent_pos[1], 299 - self.parent_pos[0]), (200, 200, 0), 1)

            image = image.astype(np.uint8)

        return pos_0, pos_1, n

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
            image[299 - i, j, :] = 0, 0, 0
            self.image_p[i, j, :] = 2

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

    def boundary(self, image, i, j):
        """
            This function plots the points that lie in the boundary.


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
        if ((j >= 399 - self.padding) or (j <= self.padding) or (i >= 299 - self.padding) or (i <= self.padding)):
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

        path = [self.goal]

        while int(self.start_score) != int(loop):

            parent_pos = self.data_with_string[loop]

            loop = self.parent_orignal_data[loop]
            index = self.data_with_string[loop]

            if index == self.start:
                break
            cv2.line(image, (parent_pos[1], 299 - parent_pos[0]), (index[1], 299 - index[0]), (255, 0, 0), 1)
            path.append(index)
            image[299 - index[0], index[1], :] = 255, 0, 0
            out.write(image)
            cv2.imshow("image", image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            image_list.append(image)
        while True:
            cv2.imshow("image", image)
            if cv2.waitKey(0) & 0xFF == ord('q'):
                break
        return path, image_list
