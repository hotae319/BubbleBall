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
from scipy.optimize import minimize, Bounds, LinearConstraint, NonlinearConstraint
import random
from math import cos, sin, pi, sqrt, atan
import os, sys
if __name__ == "__main__":
    #from obj_class import Ball, metalBlock, woodBlock, powerUps
    from physics.simple_models import Ball2LineValue, Ball2Circle
else:
    #from . obj_class import Ball, metalBlock, woodBlock, powerUps
    from . physics.simple_models import Ball2LineValue, Ball2Circle


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
    # id_pick : predicted traj's idx
    id_pick = int(num_pred_traj*dist_list[idx_local]/sum_dist)
    guide_path_local = guide_path[0:idx_local+1] 
    # direction of the end element of local guide path, angle of atan(y/x)
    direction_end = atan((guide_path[idx_local+1][1]-guide_path[idx_local][1])/(guide_path[idx_local+1][0]-guide_path[idx_local][0]))
    x_pred_max = max(x_predicted_traj_new)
    y_pred_max = max(y_predicted_traj_new)
    x_guide_max = guide_path_local[idx_local][0]
    y_guide_max = guide_path_local[idx_local][1]
    x_local = max(x_pred_max,x_guide_max)
    y_local = max(y_pred_max,y_guide_max)
    return guide_path_local, direction_end, x_local, y_local

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

def fobj(x,*y):
    # Common things irrelevant to the type of block
    # y[0] = ball.x, y[1] = ball.y, y[2] = ball.vx, y[3] = ball.vy, y[4] = ball.r
    # y[5] = guide_path_local[end][0] : x of guide path, y[6] : y of guide path
    # y[7] : direction of v, vx/vy of guide path
    # y[8] : block type, y[9] : block state [x,y,h,w,rot]
    if y[8] == "rectangle":
        # 1) retangle
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w
        #  l should be lower than y[9][3] = w 
        state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4],x[0],x[1],x[2],x[3],x[4])
    elif y[8] == "circle":
        # 2) circle
        # x[0] = x, x[1] = y, x[2] = vx, x[3] = vy, x[4] = w
        # r is given as y[9][2]/2
        state_ball, _ = Ball2CircleValue(y[0],y[1],y[2],y[3],y[4],y[9][2]/2,x[0],x[1],x[2],x[3],x[4])    
    if abs(state_ball[2]) <0.01: # if ball.vx is too low
        direction = pi/2
    else:
        direction = atan(state_ball[3]/state_ball[2])     
    # tracking error
    ref = [y[5],y[6],y[7]]    
    error = (state_ball[0]-ref[0])**2+(state_ball[1]-ref[1])**2+(direction-ref[2])**2    

    return error


def FindOptimalInput(guide_path_local, direction_end, block_type, block_state, s_ball_ini):
    # Input : guide_path_local, block_type, block_state[x,y,h,w,rot], s_ball_ini (x,y,vx,vy)
    # Output : optimal inputs to minize the tracking error based on the simple model
    xball = s_ball_ini[0]
    yball = s_ball_ini[1]
    vxball = s_ball_ini[2]
    vyball = s_ball_ini[3]
    rball = s_ball_ini[4]
    ref = (guide_path_local[-1][0],guide_path_local[-1][1], direction_end)
    # Given parameters
    # y = (ball(5),ref(3),block type, block state)
    y = (s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],s_ball_ini[4], ref[0],ref[1],ref[2], block_type, block_state)        
    if block_type == "rectangle":
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w
        x0 = [10,0,0,0,0]        
        # bound : 0 < l < width, -90<theta<90
        search_bound = Bounds([0,-90,-np.inf,-np.inf,-np.inf],[block_state[3],90,np.inf,np.inf,np.inf])
        # nonlinear constr : l and theta have same sign
        f_nonlin = lambda x:x[0]*x[1]
        nonlin_constr = NonlinearConstraint(f_nonlin,0,np.inf)
        # solve optimization
        res = minimize(fobj ,x0, args = y, bounds = search_bound, constraints = nonlin_constr)
        print(res)
        u_input = res.x
    elif block_type == "circle":
        # x[0] = x, x[1] = y, x[2] = vx, x[3] = vy, x[4] = w
        x0 = [xball+block_state[2]/2,yball+block_state[2]/2,0,0,0]
        # bound : 0 < l < width, -90<theta<90
        search_bound = Bounds([0,-90,-np.inf,-np.inf,-np.inf],[block_state[3],90,np.inf,np.inf,np.inf])
        # nonlinear constr : l and theta have same sign
        f_nonlin = lambda x:x[0]*x[1]
        nonlin_constr = NonlinearConstraint(f_nonlin,0,np.inf)
        # solve optimization
        res = minimize(fobj ,x0, args = y, bounds = search_bound, constraints = nonlin_constr)
        print(res)
        u_input = res.x

    return u_input

if __name__=="__main__":
    u = FindOptimalInput([[60,80],[100,100]], 0.3, "rectangle", [0,0,50,150,0],[60,80,0,5,15])
    print(type(u))







