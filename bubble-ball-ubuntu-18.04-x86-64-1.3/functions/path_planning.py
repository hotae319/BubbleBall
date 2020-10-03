'''
    @ Author : Hotae Lee
    @ Date : 06/06/2020
    @ Function : Identify all environments, path planning 
    @ Parameters : 
    @ Variables:  
    @ Retruns :  
    @ Description : 
    @ TODO : 
'''

import numpy as np
import matplotlib.transforms as trans
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from math import sqrt
if __name__ == "__main__":
    from planning_algo import prm as planning
    from parsing_movableobjects_levels import parsing_objects, run_simulation
else:
    from . planning_algo import prm as planning
    from functions.parsing_movableobjects_levels import parsing_objects, run_simulation
import os, sys
# add the absolute path of /parsing_bubbleball
#sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
    

def path_planning(level, n_sample, k = 6, map_size = [0,500,0,500], p_start = [0,0]): 
    '''
    Args : 
    Returns : prm(PRM class), shortest_path, sampling_list
    '''
    level_select = level
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
    num_samples = n_sample  
    if p_start == [0,0]:
        p_start = s_total[0][0:2] # ball's position
        p_start[0] += 15
        p_start[1] += 15
    else:
        p_start[0] += 15
        p_start[1] += 15
    p_end = s_total[1][0:2]
    p_end[0] += 19 # the center w,h = (38,50)
    p_end[1] += 25 
    prm = planning.PRM(map_size, s_grd_list, p_start, p_end)
    sampling_list = prm.Sampling(num_samples)
    prm.Addstartend()
    # Dicide k closest neighborhoods    
    connect_line = prm.ConnectDots(k)
    explored_path, shortest_path, came_from = prm.Astar(connect_line)
    print("explored_path : {}".format(explored_path))
    print("shortest path : {}".format(shortest_path))

    '''
    ---------------------------------------------------------------------
    Draw the figure (We can make this be a function)
    ---------------------------------------------------------------------
    '''
    # Draw path planning figure
    fig1, ax1 = plt.subplots()
    ax1.axis('scaled')
    ax1.set(xlim = (0-1,map_size[1]+1), ylim = (map_size[3]+1,0-1))
    
    # Prepare the PRM figure
    xpath = [shortest_path[i][0] for i in range(len(shortest_path))]
    ypath = [shortest_path[i][1] for i in range(len(shortest_path))]
    xsample = [sampling_list[i][0] for i in range(len(sampling_list))]
    ysample = [sampling_list[i][1] for i in range(len(sampling_list))]

    # Draw all connected lines
    for j in range(len(connect_line)):
        x = [connect_line[j][0][0],connect_line[j][1][0]]
        y = [connect_line[j][0][1],connect_line[j][1][1]]
        #ax1.plot(x,y, color = 'gray')
    # Draw all the PRM sampling points
    #ax1.scatter(xsample,ysample, s = 10, c = 'r')

    # Draw the shortest path
    ax1.plot(xpath,ypath, c = 'b')

    # Draw the grounds and the ball / flag
    for s_grd in s_grd_list:
        ts = ax1.transData
        tr = trans.Affine2D().rotate_deg_around(s_grd[0]+s_grd[2]/2,s_grd[1]+s_grd[3]/2, s_grd[4])
        t = tr + ts # tr + ts (order is important)
        rect = patches.Rectangle((s_grd[0],s_grd[1]),s_grd[2],s_grd[3], edgecolor='k', facecolor="k", transform = t)
        ax1.add_patch(rect)
    ball = plt.Circle((s_total[0][0]+15,s_total[0][1]+15),15, facecolor = 'blue') # +15 means the ball's center
    goal = patches.Rectangle((s_total[1][0],s_total[1][1]),38,50,edgecolor='g', facecolor="none")
    ax1.add_artist(ball)
    ax1.add_patch(goal)
    ax1.plot()
    #plt.show()
    return prm, shortest_path, sampling_list, ax1

def re_planning(prm, level, shortest_path_pre, k = 6, n_exclude = 1):
    '''
    Args : 
    Returns : shortest_path, sampling_list
    '''
    plt.show(block = False)
    level_select = level
    id_grd, s_grd_list, s_total, id_total, n_total, movable_ID, ID_dict, ID_state_matching = parsing_objects(level_select)
    '''
    ------------------------------------------------------
    Need to change the previous path to a new path
    ------------------------------------------------------
    '''
    # Exclude the previous shortest path    
    map_size = prm.map_size
    shortest_path = shortest_path_pre
    sampling_list = prm.ExcludePoints(shortest_path, n_exclude)
    prm.Addstartend()
    # Dicide k closest neighborhoods    
    connect_line = prm.ConnectDots(k)
    explored_path, shortest_path, came_from = prm.Astar(connect_line)
    print("explored_path : {}".format(explored_path))
    print("shortest path : {}".format(shortest_path))

    '''
    ---------------------------------------------------------------------
    Draw the figure (We can make this be a function)
    ---------------------------------------------------------------------
    '''
    # Draw path planning figure 
    fig, ax = plt.subplots()
    ax.axis('scaled')
    ax.set(xlim = (0-1,map_size[1]+1), ylim = (map_size[3]+1,0-1))
    
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
        rect = patches.Rectangle((s_grd[0],s_grd[1]),s_grd[2],s_grd[3], edgecolor='k', facecolor="None", transform = t)
        ax.add_patch(rect)
    ball = plt.Circle((s_total[0][0]+15,s_total[0][1]+15),15, facecolor = 'blue') # +15 means the ball's center
    goal = patches.Rectangle((s_total[1][0],s_total[1][1]),38,50,edgecolor='g', facecolor="none")
    ax.add_artist(ball)
    ax.add_patch(goal)
    ax.plot()   
    #plt.show()

    return shortest_path, sampling_list

if __name__ == "__main__":
    level = 9
    n_sample = 500
    map_size = [0,500,0,700]
    k1 = 6
    k2 = 10
    n_exclude = 1
    prm, shortest_path, sampling_list, ax1 = path_planning(level, n_sample, k1 , map_size)    
    #re_planning(prm, level, shortest_path, k2, n_exclude)    
    print(shortest_path)
    plt.show()