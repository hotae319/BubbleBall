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

level_select = 2

# Identify the start/end point
id_grd, s_grd_list, s_total, id_total, n_total, movable_ID, ID_dict, ID_state_matching = parsing_objects(level_select)
print("ground is {}".format(s_grd_list))
print("type {}".format(type(s_grd_list[0][0])))
# s_total, id_total(0: ball, 1:flag, ...)
print("start is {0} and end is {1}".format(s_total[0], s_total[1])) 

# path planning figure
fig, ax = plt.subplots()
ax.set(xlim = (0,500), ylim = (500,0))
ax.axis('scaled')

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

# PRM algorithm
n_prm = 30

