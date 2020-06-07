'''
	@ Author : Hotae Lee
	@ Date : 06/02/2020
	@ Function : Identify all environments, path planning and finding kep direction and acceptable region 
	@ Parameters : 
	@ Variables:  
	@ Retruns :  
	@ Description : 
	@ TODO : 
'''

import numpy as np
from parsing_movableobjects_levels import parsing_objects, run_simulation
from log2matrix import logging_trajectory
import matplotlib.transforms as trans
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from math import sqrt
import planning_algo.prm as planning


level_select = 21

# Identify the start/end point
# s_total[0] : ball, s_total[1] : flag , id_metal, id_wood, id_speedupu, id_speedupd, id_speedupl, id_speedupr, id_slowdown, id_gravity, id_gravitydown, id_spring, id_teleport
id_grd, s_grd_list, s_total, id_total, n_total, movable_ID, ID_dict, ID_state_matching = parsing_objects(level_select)

print("ground is {}".format(s_grd_list))
print("type {}".format(type(s_grd_list[0][0])))
# s_total, id_total(0: ball, 1:flag, ...)
print("start is {0} and end is {1}".format(s_total[0], s_total[1])) 



'''
-----------------------------------------------------------------------------------
PRM algorithm
-----------------------------------------------------------------------------------
'''
# Initialize 
num_samples = 400
map_size = [0,500,0,500]
p_start = s_total[0]
p_end = s_total[1]
prm = planning.PRM(map_size, s_grd_list, p_start, p_end)
sampling_list = prm.Sampling(num_samples)
prm.Addstartend()
# Dicide k closest neighborhoods
k = 6
connect_line = prm.ConnectDots(k)
explored_path, shortest_path, came_from = prm.Astar(connect_line)
print("explored_path : {}".format(explored_path))
print("shortest path : {}".format(shortest_path))

# Draw path planning figure
fig, ax = plt.subplots()
ax.set(xlim = (0-1,map_size[1]+1), ylim = (map_size[3]+1,0-1))
ax.axis('scaled')

# Prepare the PRM figure
xpath = [shortest_path[i][0] for i in range(len(shortest_path))]
ypath = [shortest_path[i][1] for i in range(len(shortest_path))]
xsample = [sampling_list[i][0] for i in range(len(sampling_list))]
ysample = [sampling_list[i][1] for i in range(len(sampling_list))]

# Draw all connected lines
for j in range(len(connect_line)):
    x = [connect_line[j][0][0],connect_line[j][1][0]]
    y = [connect_line[j][0][1],connect_line[j][1][1]]
    ax.plot(x,y, color = 'gray')
# Draw all the PRM sampling points
ax.scatter(xsample,ysample, s = 10, c = 'r')

# Draw the shortest path
ax.plot(xpath,ypath, c = 'b')

# Draw the grounds and the ball / flag
for s_grd in s_grd_list:
	ts = ax.transData
	tr = trans.Affine2D().rotate_deg_around(s_grd[0]+s_grd[2]/2,s_grd[1]+s_grd[3]/2, s_grd[4])
	t = tr + ts # tr + ts (order is important)
	rect = patches.Rectangle((s_grd[0],s_grd[1]),s_grd[2],s_grd[3], edgecolor='k', facecolor="k", transform = t)
	ax.add_patch(rect)
ball = plt.Circle((s_total[0][0],s_total[0][1]),15)
goal = patches.Rectangle((s_total[1][0],s_total[1][1]),38,50,edgecolor='g', facecolor="none")
ax.add_artist(ball)
ax.add_patch(goal)
ax.plot()
plt.show()