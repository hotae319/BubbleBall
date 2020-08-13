'''
    @ Author : Hotae Lee
    @ Date : 08/12/2020
    @ Function : Find a local region, which is parameterized and able to be adjusted easily 
    @ Parameters : 
    @ Variables:  
    @ Retruns :  
    @ Description : 
    @ TODO : 
'''
import numpy as np
import random
from math import cos, sin, pi, sqrt
if __name__ == "__main__":
    from obj_class import Ball, metalBlock, woodBlock, powerUps
    from simple_models import Ball2Line, Ball2Circle
else:
    from . obj_class import Ball, metalBlock, woodBlock, powerUps
    from . simple_models import Ball2Line, Ball2Circle


def SelectLocalRegion(guide_path, x_predicted_traj, y_predicted_traj, error_threshold = 0.1):
    # predicted traj : xtraj = [1,2,...], ytraj = [2,3,...]
    # guide_path(shortest_path): [[1,2][2,3]...]
    num_pred_traj = len(predicted_traj)
    num_guide_path = len(guide_path)
    dist_list = []
    error_list = []
    sum_dist = 0    
    x_predicted_traj_new = []
    y_predicted_traj_new = []
    for i in range(num_guide_path):
        xdist = guide_path[i][0]-guide_path[i+1][0]
        ydist = guide_path[i][1]-guide_path[i+1][1]
        dist = sqrt(xdist**2+ydist**2)
        sum_dist += dist
        dist_list.append(sum_dist)      
    # pick the pts of predicted traj to compute the tracking error
    for i in range(num_guide_path):
        id_pick = int(num_pred_traj*dist_list[i]/sum_dist)
        x_predicted_traj_new.append(x_predicted_traj[id_pick])
        y_predicted_traj_new.append(y_predicted_traj[id_pick])
        error_list.append(sqrt((guide_path[i][0]-x_predicted_traj_new[i])**2+(guide_path[i][1]-y_predicted_traj_new[i])**2))
    idx_local = 0
    while error_list[idx_local] < error_threshold:
        idx_local += 1
    # decide the local region
    id_pick = int(num_pred_traj*dist_list[idx_local]/sum_dist)
    guide_path_local = guide_path[0:idx_local+1] 
    x_pred_max = max(x_predicted_traj_new)
    y_pred_max = max(y_predicted_traj_new)
    x_guide_max = guide_path_local[idx_local][0]
    y_guide_max = guide_path_local[idx_local][1]
    x_local = max(x_pred_max,x_guide_max)
    y_local = max(y_pred_max,y_guide_max)
    return guide_path_local, x_local, y_local, idx_local, id_pick

def SortoutEnv(x_local, y_local, s_grd):
    # ground type is only rectangle, so we don't need ID or type here
    num_grd = len(s_grd)
    env_local = []
    for i in range(num_grd):
        if s_grd[i][0] <= x_local and s_grd[i][1]<=y_local:
            env_local.append(s_grd[i])
    return env_local

def PickMainBlock(movable_ID, ID_dict, guide_path_local, env_local):
    # for now they just randomly pick one block as a main block
    # We can update this with physics or classification network
    num_movable = len(movable_ID)
    random.shuffle(movable_ID)
    order2pick = movable_ID
    return order2pick

def FindOptimalInput(guide_path_local, block_type, s_ball_ini):
    # Input : guide_path_local, block_type, s_ball_ini (x,y,vx,vy)
    # Output : optimal inputs to minize the tracking error based on the simple model
    ball = Ball(s_ball_ini[0],s_ball_ini[1].s_ball_ini[2].s_ball_ini[3])
    if block_type == "rectangle":
        # optimize here
        u_input = 1
    return u_input







