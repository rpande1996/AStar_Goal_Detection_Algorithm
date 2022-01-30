import cv2
import numpy as np
from exploration import exploration_r

image = 255 * np.ones((300, 400, 3))

parent_orignal_data = []

# Inputting coordinates

print(" Start coordinates ")
s_y = input(" x_coordinate : ")
s_x = input(" y_coordinate : ")
theta_s = int((input(" theta_start : ")))
print(" Goal coordinates ")
g_y = input(" x_coordinate : ")
g_x = input(" y_coordinate : ")
theta_g = int(input(" theta_goal : "))
print(" Other parameters ")
step = int(input(" step size : "))
radius = int(input(" radius size : "))
clearance = int(input(" clearance size : "))

start = [int(s_x), int(s_y)]
goal = [int(g_x), int(g_y)]
i = 0
image = image.astype(np.uint8)

# object of class
exploration = exploration_r(start, goal, step, theta_s, theta_g, clearance, radius)

exploration.obstacles_form(image)

theta_d = 0
out = cv2.VideoWriter('../media/output/AStar_exploration.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 500, (400, 300))

image_list = []

state = exploration.obstacle_prone_area(image)
counter = 1

while state:
    i = i + 1
    counter = 2

    pos_0, pos_1, theta_d = exploration.frontier_list(image)
    out.write(image)
    image_list.append(image)  # appending all image frames in list
    # calls all moves

    image, pos_r_0, pos_r_1 = exploration.action_space(image, pos_0, pos_1, theta_d)
    if exploration.goal_reached():
        break

    exploration.expanding(pos_0, pos_1, theta_d)  # checks if node has been expanded/visited or not or not

if counter == 2:
    path, image_list = exploration.backtracking(image, out, image_list)
else:
    print("Goal or start point in the obstacle prone area")
out.release()
cv2.destroyAllWindows()
