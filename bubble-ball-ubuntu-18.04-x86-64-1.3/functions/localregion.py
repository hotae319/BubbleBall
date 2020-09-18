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
import numdifftools as nd
from matplotlib import pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as trans
from scipy.optimize import minimize, Bounds, LinearConstraint, NonlinearConstraint
import random
from math import cos, sin, pi, sqrt, atan
import time
import os, sys
if __name__ == "__main__":
    from planning_algo.utils import GetIntersectPt, CheckInside, RotatePts, LineFrom2pts, LineInequality, GetDistancePt2Line, GetLinesFromBlock, CheckInsideInequality, CheckIntersectInequality, CheckIntersectPolygon, LineFromState, GetDistancePt2Block, GetDistanceBlock2Block, GetFootPerpendicular, GetFootVertical2block
    from parsing_movableobjects_levels import parsing_objects, run_simulation, logging_trajectory
    from physics.obj_class import Ball, metalBlock, woodBlock, powerUps
    from physics.simple_models import Ball2LineValue, Ball2CircleValue, BallinAirValue, BallinEnvValue
    from physics.common.const import g, dt
else:
    from . planning_algo.utils import GetIntersectPt, CheckInside, RotatePts, LineFrom2pts, LineInequality, GetDistancePt2Line, GetLinesFromBlock, CheckInsideInequality, CheckIntersectInequality, CheckIntersectPolygon, LineFromState, GetDistancePt2Block, GetDistanceBlock2Block, GetFootPerpendicular, GetFootVertical2block
    from . parsing_movableobjects_levels import parsing_objects, run_simulation, logging_trajectory
    from . physics.obj_class import Ball, metalBlock, woodBlock, powerUps
    from . physics.simple_models import Ball2LineValue, Ball2CircleValue, BallinAirValue, BallinEnvValue
    from . physics.common.const import g, dt


def SelectLocalRegion(guide_path, x_predicted_traj, y_predicted_traj, s_grd_list, error_threshold = 200):
    # predicted traj : xtraj = [1,2,...], ytraj = [2,3,...]
    # guide_path(shortest_path): [[1,2][2,3]...]
    num_pred_traj = len(x_predicted_traj)
    num_guide_path = len(guide_path)
    dist_list_pred = []
    dist_list = []
    error_list = []
    sum_dist_pred = 0
    sum_dist = 0   
    x_predicted_traj_new = []
    y_predicted_traj_new = []
    # save the intervals of pred_traj and guide_path
    for i in range(num_pred_traj-1):
        xdist_pred = x_predicted_traj[i]-x_predicted_traj[i+1]
        ydist_pred = y_predicted_traj[i]-y_predicted_traj[i+1]
        dist_pred = sqrt(xdist_pred**2+ydist_pred**2)
        sum_dist_pred += dist_pred
        dist_list_pred.append(sum_dist_pred)
    for i in range(num_guide_path-1):
        xdist = guide_path[i][0]-guide_path[i+1][0]
        ydist = guide_path[i][1]-guide_path[i+1][1]
        dist = sqrt(xdist**2+ydist**2)
        sum_dist += dist
        dist_list.append(sum_dist)      

    #print("dist_list_pred {}".format(dist_list_pred))
    # pick the pts of predicted traj to compute the tracking error
    # The entire predicted traj. is split into several parts which are proportional to interval of Tg
    for i in range(num_guide_path-1):        
        l = dist_list[i]/sum_dist*sum_dist_pred
        id_pick = 1
        while not (dist_list_pred[id_pick-1]<= l and dist_list_pred[id_pick] > l) and id_pick < len(dist_list_pred)-1:
            id_pick += 1
        #id_pick = int(num_pred_traj*dist_list[i]/sum_dist)-1 # consider only the number of elements, not intervals
        #print("l, dist_list_pred id_pick, len{} {} {} {}".format(l, dist_list_pred[id_pick], id_pick, len(dist_list_pred)))
        x_predicted_traj_new.append(x_predicted_traj[id_pick])
        y_predicted_traj_new.append(y_predicted_traj[id_pick])
        error_list.append(sqrt((guide_path[i][0]-x_predicted_traj_new[i])**2+(guide_path[i][1]-y_predicted_traj_new[i])**2))
    # new index for new list (the number is same with the guide path)
    idx_local_start = 0
    idx_local_end = 0
    print("error list {}".format(error_list))    
    print("traj new {} , {}".format(x_predicted_traj_new, y_predicted_traj_new))
    while error_list[idx_local_start] < error_threshold/18:
        idx_local_start += 1
    while error_list[idx_local_end] < error_threshold and idx_local_end < len(error_list)-1:
        idx_local_end += 1
    # Decide the local region
    # id_pick : predicted traj's idx (outside the function, we need to choose s_ball_init)
    #idx_pick_start = int(num_pred_traj*dist_list[idx_local_start]/sum_dist)
    #idx_pick_end = int(num_pred_traj*dist_list[idx_local_end]/sum_dist)
    idx_pick_start = y_predicted_traj.index(y_predicted_traj_new[idx_local_start]) # if it is repeated, return the first value
    idx_pick_end = y_predicted_traj.index(y_predicted_traj_new[idx_local_end])

    guide_path_local = guide_path[idx_local_start:idx_local_end+1] 
    print("guide_path_local {}".format(guide_path_local))
    # direction of the end element of local guide path, angle of atan(y/x)
    if guide_path[idx_local_end+1][0]-guide_path[idx_local_end][0] == 0:
        direction_end = pi/2*np.sign(guide_path[idx_local_end+1][1]-guide_path[idx_local_end][1])
    else:
        direction_end = atan((guide_path[idx_local_end+1][1]-guide_path[idx_local_end][1])/(guide_path[idx_local_end+1][0]-guide_path[idx_local_end][0]))
    # It can be changed to the total prediction traj
    x_pred_max = max(x_predicted_traj[idx_pick_start], x_predicted_traj[idx_pick_end])
    y_pred_max = max(y_predicted_traj[idx_pick_start], y_predicted_traj[idx_pick_end])
    x_guide_max = max(guide_path_local[idx_local_end-idx_local_start][0], guide_path_local[0][0])
    y_guide_max = max(guide_path_local[idx_local_end-idx_local_start][1], guide_path_local[0][1])
    x_local_max = max(x_guide_max, x_pred_max)
    y_local_max = max(y_pred_max, y_guide_max)
    #x_local = max(x_pred_max,x_guide_max)
    x_local_min = min(guide_path_local[0][0],guide_path_local[-1][0], x_predicted_traj[idx_pick_start], x_predicted_traj[idx_pick_end])
    y_local_min = min(guide_path_local[0][1],guide_path_local[-1][1], y_predicted_traj[idx_pick_start], y_predicted_traj[idx_pick_end])
    # Adjust the local region's y value
    
    s_grd_local = SortoutEnv(x_local_max, y_local_max, x_local_min, y_local_min, s_grd_list) 
    
    print(x_predicted_traj_new)
    while y_local_max <= 500 and not s_grd_local :
    #while not s_grd_local or (CheckCondition(s_grd_local, y_pred_max, y_guide_max) and y_local_max <= 500):
        y_local_max += 100        
        s_grd_local = SortoutEnv(x_local_max, y_local_max, x_local_min, y_local_min, s_grd_list)
    return guide_path_local, direction_end, x_local_max, y_local_max, idx_pick_start, idx_pick_end
def SortedTrajInLocal(ball_traj, local_region):
    # local_region [x_local_max, y_local_max, xregion, yregion]
    # ball_traj = [[vx,vy,w,x,y,rot],...]
    traj_sorted = []
    idx_sorted = []
    for i in range(len(ball_traj)):
        if ball_traj[i][3] <= x_local_max and ball_traj[i][3] >= xregion and ball_traj[i][4] <= y_local_max and ball_traj[i][4] >= yregion:
            traj_sorted.append(ball_traj[i])
            idx_sorted.append(i)
    return traj_sorted, idx_sorted

def CheckCondition(s_grd_list, y_pred_max, y_guide_max):
    print(s_grd_list)
    print(max(y_pred_max, y_guide_max))
    bool_condition = True
    for i in range(len(s_grd_list)):
        if s_grd_list[i][1] > max(y_pred_max, y_guide_max) + 50:
            bool_condition = False
    return bool_condition

def SortoutEnv(x_local_max, y_local_max, x_local_min, y_local_min, s_grd_list):
    # ground type is only rectangle, so we don't need ID or type here
    num_grd = len(s_grd_list)
    env_local = []    
    state_region = [x_local_min, y_local_min, x_local_max-x_local_min, y_local_max-y_local_min, 0]
    pts_region = RotatePts(state_region, "ground")
    print("state region, pts_region : {}, {}".format(state_region, pts_region))
    for i in range(num_grd):
        print("ground {}".format(s_grd_list[i]))
        pts_grd = RotatePts(s_grd_list[i], "ground") # we can add groundtriangle
        # we can use CheckOverlap function here
        bool_overlap, _ = CheckOverlap(s_grd_list[i], state_region) 
        print("bool overlap {}",format(bool_overlap))
        if bool_overlap == True:
            env_local.append(s_grd_list[i])
        # if (pts[1][0] > x_local_max and pts[2][0] > x_local_max) or (pts[1][0] < x_local_min and pts[2][0] < x_local_min):
        #     pass# the block is outside the region
        #     print("outside1")
        #     print(pts[1][0], pts[2][0], x_local_max, x_local_min)
        # elif (pts[1][1] > y_local_max and pts[2][1] > y_local_max) or (pts[1][1] < y_local_min and pts[2][1] < y_local_min):
        #     print("outside2")
        #     print(pts[1][1], pts[2][1], y_local_max, y_local_min)
        #     pass#  the block is outside the region
        # else:
        #     env_local.append(s_grd_list[i])

        print("\n")
    return env_local

def PickMainBlock(movable_ID, ID_dict, guide_path_local, env_local):
    # for now they just randomly pick one block as a main block
    # We can update this with physics or classification network
    num_movable = len(movable_ID)
    random.shuffle(movable_ID)
    order2pick = movable_ID
    return order2pick

def GetError(ref, state_ball):
    if abs(state_ball[2]) <0.01: # if ball.vx is too low
        direction = pi/2
    else:
        direction = atan(state_ball[3]/state_ball[2])    
    error = (state_ball[0]-ref[0])**2+(state_ball[1]-ref[1])**2+100*(direction-ref[2])**2 
    error_vector = [ref[0]-state_ball[0], ref[1]-state_ball[1], ref[2]-direction]
    return error, error_vector

def fobj(x,*y):
    # x is the optimization variable

    # Common things irrelevant to the type of block
    # y[0] = ball.x, y[1] = ball.y, y[2] = ball.vx, y[3] = ball.vy, y[4] = ball.r
    # y[5] = guide_path_local[end][0] : x of guide path, y[6] : y of guide path
    # y[7] : direction of v, vx/vy of guide path
    # y[8] : block type, y[9] : block state [x,y,w,h,rot]
    # y[10] : env_local [[x,y,w,h,rot],[x,y,w,h,rot],...]

    # tracking error (ref = [x,y,dir])
    ref = [y[5],y[6],y[7]]  

    # All rollouts consist of "gravity or environment -> input -> gravity or environment"
    if y[8] in ("metalrectangle" , "woodrectangle"):
        # 1) retangle
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w
        #  l should be lower than y[9][3] = w 
        state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4],x[0],x[1],x[2],x[3],x[4])
    elif y[8] == "woodcircle" or y[8] == "metalcircle":
        # 2) circle
        # x[0] = x, x[1] = y, x[2] = vx, x[3] = vy, x[4] = w
        # r is given as y[9][2]/2
        state_ball_collision = Ball2CircleValue(y[0],y[1],y[2],y[3],y[4],y[9][2]/2,x[0],x[1],x[2],x[3],x[4])
        x_distance = abs(ref[0]-y[0])
        y_distance = abs(ref[1]-y[1])
        state_ball = BallinAirValue(state_ball_collision[0],state_ball_collision[1],state_ball_collision[2],state_ball_collision[3],x_distance,y_distance)    
    elif y[8] in ("metalrectangle" , "woodrectangle"):
        # if we have to consider x1 at the same time
        state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4],x[0],x[1],x[2],x[3],x[4])
    # error tracking cost
    error,_ = GetError(ref, state_ball)
    # if abs(state_ball[2]) <0.01: # if ball.vx is too low
    #     direction = pi/2
    # else:
    #     direction = atan(state_ball[3]/state_ball[2])  
    # error = (state_ball[0]-ref[0])**2+(state_ball[1]-ref[1])**2+(direction-ref[2])**2   

    # sum_penalty = 0 
    # block_state = y[9]
    # xball = y[0]
    # yball = y[1]
    # vxball = y[2]
    # vyball = y[3]
    # rball = y[4]
    # def pts_block(x):     
    #     l = x[0]
    #     theta = x[1]/180*pi
    #     width = block_state[2]
    #     height = block_state[3]
    #     u_actual = [xball+rball-rball*sin(theta)+l*cos(theta)-width/2-width/2*cos(theta)-height/2*sin(theta), yball+rball+rball*cos(theta)+l*sin(theta)-height/2-width/2*sin(theta)+height/2*cos(theta), x[1]]
    #     pts_block = RotatePts([u_actual[0],u_actual[1],width, height, u_actual[2]], block_type)
    #     return pts_block  
    # for env_element in env_local:
    # #env_element = env_local[2]    
    #     width_element = env_element[2] + 30
    #     height_element = env_element[3] + 30
    #     state_element = [env_element[0], env_element[1], width_element, height_element, env_element[4]]
    #     pts = RotatePts(state_element, "ground")
    #     i_list = [0,1,2,3]
    #     for i in i_list:
    #         for j in range(4):       
    #             # [eq1, eq2]
    #             f_interference3 = CheckIntersectInequality(pts_block(x)[i], pts_block(x)[i-1], pts[j], pts[j-1])
    #             sum_penalty += max(0,f_interference3[0])*10 + max(0,f_interference3[1])*10
    # error = error + sum_penalty 
    return error

def fobj_constr(x,*y):
    # x is the optimization variable

    # Common things irrelevant to the type of block
    # y[0] = ball.x, y[1] = ball.y, y[2] = ball.vx, y[3] = ball.vy, y[4] = ball.r
    # y[5] = guide_path_local[end][0] : x of guide path, y[6] : y of guide path
    # y[7] : direction of v, vx/vy of guide path
    # y[8] : block type, y[9] : block state [x,y,w,h,rot]
    # y[10] : env_type_pre, y[11] : env_state_pre, y[12] : env_type_post, y[13] : env_state_post
    # y[14] : local_region [x_local_max, y_local_max, xregion, yregion]

    # tracking error (ref = [x,y,dir])
    ref = [y[5],y[6],y[7]]  

    # All rollouts consist of "gravity or environment -> input -> gravity or environment"
    # For f1 (first region before control input)    
    if y[10] == "ground":
        state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4], x[5]/cos(y[11][4]/180*pi), y[11][4], 0, 0, 0)
    else: #elif y[10] == "air":
        if abs(y[2]) < 0.5:
            state_ball = BallinAirValue(y[0],y[1],y[2],y[3],0,x[5])
        else:
            state_ball = BallinAirValue(y[0],y[1],y[2],y[3],x[5])
    xpre = x[5]
    # For fu (control region)
    if y[8] in ("metalrectangle" , "woodrectangle"):
        # 1) retangle
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w, x[5] = lx (if lx=0, ly)
        #  l should be lower than y[9][3] = w 
        state_ball, _ = Ball2LineValue(state_ball[0],state_ball[1],state_ball[2],state_ball[3],y[4],x[0],x[1],x[2],x[3],x[4])
        xu = x[0]*cos(x[1]/180*pi)
    elif y[8] == "woodcircle" or y[8] == "metalcircle":
        # 2) circle
        # x[0] = x, x[1] = y, x[2] = vx, x[3] = vy, x[4] = w
        # r is given as y[9][2]/2
        state_ball_collision = Ball2CircleValue(state_ball[0],state_ball[1],state_ball[2],state_ball[3],y[4],y[9][2]/2,x[0],x[1],x[2],x[3],x[4])
        x_distance = abs(ref[0]-y[0])
        y_distance = abs(ref[1]-y[1])
        state_ball = BallinAirValue(state_ball_collision[0],state_ball_collision[1],state_ball_collision[2],state_ball_collision[3],x_distance,y_distance)    
        xu = y[9][2]
    # For f2 (last region after control input)
    xpost = y[14][0] - xu - xpre  
    # if y[12] == "ground":
    #     state_ball, _ = Ball2LineValue(state_ball[0],state_ball[1],state_ball[2],state_ball[3], y[4], xpost/cos(y[13][4]/180*pi), y[13][4], 0, 0, 0)
    # else: #elif y[12] == "air": we need to decide more expressive functions based on observations 
    #     state_ball= BallinAirValue(state_ball[0],state_ball[1],state_ball[2],state_ball[3],xpost)

    if abs(state_ball[2]) <0.01: # if ball.vx is too low
        direction = pi/2
    else:
        direction = atan(state_ball[3]/state_ball[2])    
    # error tracking cost
    error = (state_ball[0]-ref[0])**2+(state_ball[1]-ref[1])**2+(direction-ref[2])**2    

    return error

def fmodel(x,*y):
    # x is the optimization variable

    # Common things irrelevant to the type of block
    # y[0] = ball.x, y[1] = ball.y, y[2] = ball.vx, y[3] = ball.vy, y[4] = ball.r
    # y[5] = guide_path_local[end][0] : x of guide path, y[6] : y of guide path
    # y[7] : direction of v, vx/vy of guide path
    # y[8] : block type, y[9] : block state [x,y,w,h,rot]

    # tracking error (ref = [x,y,dir])
    ref = [y[5],y[6],y[7]]  

    # All rollouts consist of "gravity or environment -> input -> gravity or environment"
    if y[8] in ("metalrectangle" , "woodrectangle"):
        # 1) retangle
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w
        #  l should be lower than y[9][3] = w 
        state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4],x[0],x[1],x[2],x[3],x[4])
    elif y[8] == "woodcircle" or y[8] == "metalcircle":
        # 2) circle
        # x[0] = x, x[1] = y, x[2] = vx, x[3] = vy, x[4] = w
        # r is given as y[9][2]/2
        state_ball_collision = Ball2CircleValue(y[0],y[1],y[2],y[3],y[4],y[9][2]/2,x[0],x[1],x[2],x[3],x[4])
        x_distance = abs(ref[0]-y[0])
        y_distance = abs(ref[1]-y[1])
        state_ball = BallinAirValue(state_ball_collision[0],state_ball_collision[1],state_ball_collision[2],state_ball_collision[3],x_distance,y_distance)    
    if abs(state_ball[2]) <0.01: # if ball.vx is too low
        direction = pi/2
    else:
        direction = atan(state_ball[3]/state_ball[2])    
    result = np.array([state_ball[0],state_ball[1],direction])
    return result

def ConvertUopt2Ureal(u_input, block_type, w_block, h_block, s_ball_ini):    
    xball = s_ball_ini[0]
    yball = s_ball_ini[1]
    vxball = s_ball_ini[2]
    vyball = s_ball_ini[3]
    rball = s_ball_ini[4]
    if block_type in ("metalrectangle" , "woodrectangle"):
        # u_input = [ l, theta, vx, vy, w] + lx
        l = u_input[0]*0.99 # for safe activation
        width = w_block
        height = h_block
        theta = u_input[1]/180*pi
        if l >=0:
            u_actual = [xball+rball-rball*sin(theta)+l*cos(theta)-width/2-width/2*cos(theta)-height/2*sin(theta), yball+rball+rball*cos(theta)+l*sin(theta)-height/2-width/2*sin(theta)+height/2*cos(theta), u_input[1]]
        else:
            u_actual = [xball+rball-rball*sin(theta)+l*cos(theta)-width/2+width/2*cos(theta)-height/2*sin(theta), yball+rball+rball*cos(theta)+l*sin(theta)-height/2+width/2*sin(theta)+height/2*cos(theta), u_input[1]]
        vel_desired = [u_input[2],u_input[3],u_input[4]]
    elif block_type == "woodcircle" or block_type ==  "metalcircle":
        # u_input = [x, y, vx, vy, w]
        u_actual = [u_input[0],u_input[1],0]
        vel_desired = [u_input[2],u_input[3],u_input[4]]
    elif block_type in ("metalrtriangle", "woodrtriangle"):
        # u_input = [ l, theta, vx, vy, w]   
        l = u_input[0]     
        theta = (u_input[1]+45)/180*pi
        u_actual = [xball-(w_block*sqrt(2)-l)*cos(theta), yball+ rball*2, theta]
    return u_actual, vel_desired

def MakeF1withTraj(traj_sorted, x1):
    # traj_sorted : [[vx,vy,w,x,y,rot],...] / x1 can be either positive or negative
    x_init = traj_sorted[0][3]
    y_init = traj_sorted[0][4]
    x_end = traj_sorted[-1][3]
    y_end = traj_sorted[-1][4]
    vx_init = traj_sorted[0][0]
    max_i = len(traj_sorted)
    if abs(vx_init) < 0.5:
        # consider y direction first
        i = 0
        while (traj_sorted[i][4] < y_init + x1 and traj_sorted[i+1][4] < y_init + x1 and i< max_i-2 and abs(traj_sorted[0][0])<0.5) or (traj_sorted[i][4] > y_init + x1 and traj_sorted[i+1][4] > y_init + x1 and i < max_i-2 and abs(traj_sorted[0][0])<0.5):
            i += 1
    else:
        i = 0
        if x_init < x_end: # left to right            
            while (traj_sorted[i][3] < x_init + x1 and traj_sorted[i+1][3] < x_init + x1 and i< max_i-2) or (traj_sorted[i][3] > x_init + x1 and traj_sorted[i+1][3] > x_init + x1 and i < max_i-2):
                i += 1
        else:
            while (traj_sorted[i][3] < x_init - x1 and traj_sorted[i+1][3] < x_init - x1 and i< max_i-2) or (traj_sorted[i][3] > x_init - x1 and traj_sorted[i+1][3] > x_init - x1 and i < max_i-2):
                i += 1        
        if i == max_i-2:
            # traj. consists of only y movement.
            i = 0
            while (traj_sorted[i][4] < y_init + x1 and traj_sorted[i+1][4] < y_init + x1 and i< max_i-2) or (traj_sorted[i][4] > y_init + x1 and traj_sorted[i+1][4] > y_init + x1 and i < max_i-2):
                i += 1
    # convert to the state type
    #print("i {}".format(i))
    state_x1 = [traj_sorted[i][3],traj_sorted[i][4],traj_sorted[i][0],traj_sorted[i][1]]
    return state_x1

def FindOptimalInput(guide_path_local, direction_end, block_type, block_state, s_ball_ini, env_local):
    # the size of the local region and the environment in the local region can be input
    # But, we want to minimize the intervention of environment.
    # Input : guide_path_local, block_type, block_state[x,y,w,h,rot], s_ball_ini (x,y,vx,vy,r)
    # Output : optimal inputs to minize the tracking error based on the simple model
    xball = s_ball_ini[0]
    yball = s_ball_ini[1]
    vxball = s_ball_ini[2]
    vyball = s_ball_ini[3]
    rball = s_ball_ini[4]
    # ref : [x,y,dir]
    ref = (guide_path_local[-1][0],guide_path_local[-1][1], direction_end)
    constr_list = []
    # Given parameters
    # y = (ball(5),ref(3),block type, block state)
    y = (s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],s_ball_ini[4], ref[0],ref[1],ref[2], block_type, block_state, env_local)        
    if block_type == "metalrectangle" or block_type == "woodrectangle":
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w
        # block_state = [x,y,w,h,rot]        
        x0 = [block_state[2]*0.9,2,0,0,0]        
        # bound : -width < l < width, -90<theta<90
        search_bound = Bounds([-block_state[2],-90,-np.inf,-np.inf,-np.inf],[block_state[2],90,np.inf,np.inf,np.inf])
        # nonlinear constr : l and theta have same sign
        f_nonlin = lambda x:x[0]
        nonlin_constr = NonlinearConstraint(f_nonlin,0,np.inf)
        constr_list.append(nonlin_constr)
        # Interference constraint  
        def pts_block(x):     
            l = x[0]
            theta = x[1]/180*pi
            width = block_state[2]
            height = block_state[3]
            u_actual = [xball+rball-rball*sin(theta)+l*cos(theta)-width/2-width/2*cos(theta)-height/2*sin(theta), yball+rball+rball*cos(theta)+l*sin(theta)-height/2-width/2*sin(theta)+height/2*cos(theta), x[1]]
            pts_block = RotatePts([u_actual[0],u_actual[1],width, height, u_actual[2]], block_type)
            return pts_block      
        # for env_element in env_local:
        #     # for radius (size growing downwards)
        #     # to avoid referring to the address of list directly
        #     width_element = env_element[2] + 30
        #     height_element = env_element[3] + 30
        #     state_element = [env_element[0], env_element[1], width_element, height_element, env_element[4]]
        #     pts = RotatePts(state_element, "ground")
        #     for i in range(4):
        #         for j in range(4):
        #             f_interference3 = lambda x: CheckIntersectInequality(pts_block(x)[i], pts_block(x)[i-1], pts[j], pts[j-1])
        #             constr_interference3 = NonlinearConstraint(f_interference3, -np.inf, 0)
        #             constr_list.append(constr_interference3)
        #for env_element in env_local:
        # env_element = env_local[2]
        # width_element = env_element[2] + 30
        # height_element = env_element[3] + 30
        # state_element = [env_element[0], env_element[1], width_element, height_element, env_element[4]]
        # pts = RotatePts(state_element, "ground")
        # i_list = [0,1,2,3]
        # for i in i_list:
        #     for j in range(4):       
        #         f_interference3 = lambda x: CheckIntersectInequality(pts_block(x)[i], pts_block(x)[i-1], pts[j], pts[j-1])
        #         constr_interference3 = NonlinearConstraint(f_interference3, -np.inf, 0)
        #         constr_list.append(constr_interference3)
        # solve optimization
        res = minimize(fobj ,x0, args = y , bounds = search_bound, constraints = constr_list, method = 'trust-constr')
        print(res)                
        u_input = res.x
    elif block_type == "woodcircle" or block_type == "metalcircle":
        # x[0] = x, x[1] = y, x[2] = vx, x[3] = vy, x[4] = w
        x0 = [xball+block_state[2]/2,yball+block_state[2]/2,0,0,0]
        # bound : we need a condition to collide and x,y should be inside the local region
        search_bound = Bounds([0,0,-np.inf,-np.inf,-np.inf],[ref[0],ref[1],np.inf,np.inf,np.inf])
        # nonlinear constr : nothing now
        #f_nonlin = lambda x:x[0]*x[1]
        #nonlin_constr = NonlinearConstraint(f_nonlin,0,np.inf)
        # solve optimization
        res = minimize(fobj ,x0, args = y, bounds = search_bound)
        print(res)
        u_input = res.x
    return u_input

def FindOptimalInputGrid(guide_path_local, direction_end, block_type, block_state, s_ball_ini, env_local):
    # the size of the local region and the environment in the local region can be input
    # But, we want to minimize the intervention of environment.
    # Input : guide_path_local, block_type, block_state[x,y,w,h,rot], s_ball_ini (x,y,vx,vy,r)
    # Output : optimal inputs to minize the tracking error based on the simple model
    xball = s_ball_ini[0]
    yball = s_ball_ini[1]
    vxball = s_ball_ini[2]
    vyball = s_ball_ini[3]
    rball = s_ball_ini[4]
    # ref : [x,y,dir]
    ref = (guide_path_local[-1][0],guide_path_local[-1][1], direction_end)
    w_main = block_state[2]
    h_main = block_state[3]
    constr_list = []
    # Given parameters

    # y = (ball(5),ref(3),block type, block state)
    y = (s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],s_ball_ini[4], ref[0],ref[1],ref[2], block_type, block_state, env_local)        
    if block_type == "metalrectangle" or block_type == "woodrectangle":
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w            
        # bound : -width < l < width, -90<theta<90, l*theta >0
        l_resolution = 20
        theta_resolution = 5
        f_obj_value_min = 10000000
        u_input_min = [w_main, 0, 0, 0, 0]
        f_obj_list = []
        u_input_list = []
        u_fobj_tuple_list = []
        # grid of l, theta
        #l_grid = range(int(w_main/2), w_main, l_resolution) # 5~6
        l_grid = range(-w_main, w_main, l_resolution)
        theta_grid = range(-80, 80, theta_resolution) # 32        
        for l_cand in l_grid:
            for theta_cand in theta_grid:
                u_input = [l_cand,theta_cand,0,0,0] 
                if block_type == "woodrectangle":
                    # Check feasibility
                    u_actual, vel_desired = ConvertUopt2Ureal(u_input, block_type, w_main, h_main, s_ball_ini)
                    main_state = [u_actual[0], u_actual[1], w_main, h_main, u_actual[2]]
                    need_adjust = AdjustmentConstraint(main_state, env_local)
                    # If feasible, compute objective function
                    if need_adjust == False:
                        f_obj_value_current = fobj(u_input,*y)   
                        uf_tuple = (u_input, f_obj_value_current)
                        u_fobj_tuple_list.append(uf_tuple)                 
                        # f_obj_list.append(f_obj_value_current)
                        # u_input_list.append(u_input)
                        if f_obj_value_current <= f_obj_value_min:
                            f_obj_value_min = f_obj_value_current
                            u_input_min = u_input
                else:
                    f_obj_value_current = fobj(u_input,*y)   
                    uf_tuple = (u_input, f_obj_value_current)
                    u_fobj_tuple_list.append(uf_tuple)                 
                    # f_obj_list.append(f_obj_value_current)
                    # u_input_list.append(u_input)
                    if f_obj_value_current <= f_obj_value_min:
                        f_obj_value_min = f_obj_value_current
                        u_input_min = u_input

    else:
        f_obj_value_min = 10000000
        u_input_min = 0
    return u_input_min, f_obj_value_min, u_fobj_tuple_list




def FindOptimalInputConstrained(guide_path_local, direction_end, block_type, block_state, s_ball_ini, env_type_pre, env_state_pre, env_type_post, env_state_post, env_local, local_region):
    # the size of the local region and the environment in the local region can be input
    # But, we want to minimize the intervention of environment.
    # Input : guide_path_local, block_type, block_state[x,y,w,h,rot], s_ball_ini (x,y,vx,vy,r), 
    #         evn_type ("ground" or "air")
    #         env_local : [[x,y,w,h,rot],[x,y,w,h,rot],..], local_region : [x_local_max, y_local_max, xregion, yregion]
    # Output : optimal inputs to minize the tracking error based on the simple model    
    xball = s_ball_ini[0]
    yball = s_ball_ini[1]
    vxball = s_ball_ini[2]
    vyball = s_ball_ini[3]
    rball = s_ball_ini[4]
    # ref : [x,y,dir]
    ref = (guide_path_local[-1][0],guide_path_local[-1][1], direction_end)
    constr_list = []
    # Given parameters
    # y = (ball(5),ref(3),block type, block state, env_type_pre, env_state_pre, env_type_post, env_state_post, localregion(4))
    y = (s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],s_ball_ini[4], ref[0],ref[1],ref[2], block_type, block_state, env_type_pre, env_state_pre, env_type_post, env_state_post, local_region)        
    if block_type == "metalrectangle" or block_type == "woodrectangle":
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w / x[5] = lx for s_mid = f1(s_in, lx) (lx = 0, ly)
        def pts_block(x):       
            if env_type_pre == "ground":
                state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4], x[5]/cos(y[11][4]/180*pi), y[11][4], 0, 0, 0)
            else: #elif y[10] == "air":
                if abs(y[2]) < 0.5:
                    state_ball = BallinAirValue(y[0],y[1],y[2],y[3],0,x[5])
                else:
                    state_ball = BallinAirValue(y[0],y[1],y[2],y[3],x[5])     
            l = x[0]
            theta = x[1]/180*pi
            width = block_state[2]
            height = block_state[3]
            u_actual = [state_ball[0]+rball-rball*sin(theta)+l*cos(theta)-width/2-width/2*cos(theta)-height/2*sin(theta), state_ball[1]+rball+rball*cos(theta)+l*sin(theta)-height/2-width/2*sin(theta)+height/2*cos(theta), x[1]]
            pts_block = RotatePts([u_actual[0],u_actual[1],width, height, u_actual[2]], block_type)
            return pts_block
        if abs(vxball) < 0.5:
            # x[5] = ly
            x0 = [block_state[2],0,0,0,0, (y_local_max-yregion)/10]     
            search_bound = Bounds([0,-90,-np.inf,-np.inf,-np.inf, 0],[block_state[2],90,np.inf,np.inf,np.inf, y_local_max-yregion])
        else:
            # block_state = [x,y,w,h,rot]        
            x0 = [block_state[2],0,0,0,0, (x_local_max-xregion)/4]        
            # bound : 0 < l < width, -90<theta<90
            search_bound = Bounds([0,-90,-np.inf,-np.inf,-np.inf, 0],[block_state[2],90,np.inf,np.inf,np.inf, x_local_max-xregion])
        # nonlinear constr : l and theta have same sign
        f_nonlin = lambda x:x[0]*x[1]
        nonlin_constr = NonlinearConstraint(f_nonlin,0,np.inf)
        constr_list.append(nonlin_constr)
        # process constraint
        # if env_type_pre == "ground":
        #     state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4], x[5]/cos(y[11][4]/180*pi), y[11][4], 0, 0, 0)
        # else: #elif y[10] == "air":
        #     state_ball, _ = BallinAirValue(y[0],y[1],y[2],y[3],x[5])

        # Interference constraint        
        for env_element in env_local:
            # for radius (size growing downwards)
            # to avoid referring to the address of list directly
            width_element = env_element[2] + 30
            height_element = env_element[3] + 30
            state_element = [env_element[0], env_element[1], width_element, height_element, env_element[4]]
            pts = RotatePts(state_element, "ground")
            # for i in range(4):
            #     f_interference1 = lambda x: CheckInsideInequality(pts, pts_block(x)[i])
            #     f_interference2 = lambda x: CheckInsideInequality(pts_block(x), pts[i])
            #     constr_interference1 = NonlinearConstraint(f_interference1, -np.inf, 0)
            #     constr_interference2 = NonlinearConstraint(f_interference2, -np.inf, 0)
            #     constr_list.append(constr_interference1)
            #     constr_list.append(constr_interference2)
            # for i in range(4):
            #     for j in range(4):
            #         f_interference3 = lambda x: CheckIntersectInequality(pts_block(x)[i], pts_block(x)[i-1], pts[j], pts[j-1])
            #         constr_interference3 = NonlinearConstraint(f_interference3, -np.inf, 0)
            #         constr_list.append(constr_interference3)

        # pts = RotatePts(env_local[2], "ground")
        # f_test = lambda x: LineInequality(pts[1],pts[2],pts_block(x)[0])
        # constr_test = NonlinearConstraint(f_test, -np.inf, 0)
        # constr_list.append(constr_test)
        # f_test = lambda x: [pts_block(x)[0][0]-450, pts_block(x)[1][0]-450, pts_block(x)[2][0]-450]
        # constr_test = NonlinearConstraint(f_test, -np.inf, 0)
        # constr_list.append(constr_test)
        # f_test = lambda x : 2-x[1]
        # constr_test = NonlinearConstraint(f_test, -np.inf, 0)
        # constr_list.append(constr_test)
        # solve optimization
        res = minimize(fobj_constr ,x0, args = y, bounds = search_bound, constraints = constr_list)
        print(res)                
        u_input = res.x
    elif block_type == "woodcircle" or block_type == "metalcircle":
        # x[0] = x, x[1] = y, x[2] = vx, x[3] = vy, x[4] = w
        x0 = [xball+block_state[2]/2,yball+block_state[2]/2,0,0,0]
        # bound : we need a condition to collide and x,y should be inside the local region
        search_bound = Bounds([0,0,-np.inf,-np.inf,-np.inf],[ref[0],ref[1],np.inf,np.inf,np.inf])
        # nonlinear constr : nothing now
        #f_nonlin = lambda x:x[0]*x[1]
        #nonlin_constr = NonlinearConstraint(f_nonlin,0,np.inf)
        # solve optimization
        res = minimize(fobj_constr ,x0, args = y, bounds = search_bound)
        print(res)
        u_input = res.x
    return u_input

def ComputeSupportForces(mainblock_type, mainblock_desired_state, main_traj,  contact_idx):
    # Compute the 2 forces or 1 force with a fixed force, given the desired state of a main block
    # Input : type("rectangle"), desired_state(x,y,w,h,rot,vx,vy,w_rot)
    #         current trajectory : main_traj([[vx,vy,w,x,y,rot],[],...,[]])
    #         contact_point : [x,y], contact_idx : index of traj. when the contact starts
    # Output : two points we will support (p1[x1,y1], p2[x2,y2]) / one point or range corresponding to given contact pt
    # the supporting forces which can compensate the movement
    w = mainblock_desired_state[2]
    h = mainblock_desired_state[3]
    if mainblock_type == "woodrectangle":
        m = 1.5*mainblock_desired_state[2]*mainblock_desired_state[3]
        I = 1/12*m*(mainblock_desired_state[2]**2+mainblock_desired_state[3]**2)
    elif mainblock_type == "woodcircle":
        m = 1.5*pi*(mainblock_desired_state[3]/2)**2
        I = 1/2*m*(mainblock_desired_state[3]/2)**2
    elif mainblock_type == "woodrtriangle":
        m = 1/2*1.5*mainblock_desired_state[2]*mainblock_desired_state[3]
        I = 1/9*m*mainblock_desired_state[3]**2


    # Compute the current forces under this traj.    
    # If we observed main_traj, we can compute force traj.
    fx_list = []
    fy_list = []
    torque_list = []
    for i in range(len(main_traj)-1):
        thetadot = main_traj[i][2]/180*pi
        theta= main_traj[i][5]/180*pi
        xcmdot_pre = main_traj[i][0]-h/2*thetadot*cos(theta)-w/2*thetadot*sin(theta)
        ycmdot_pre = main_traj[i][1]+h/2*thetadot*sin(theta)+w/2*thetadot*cos(theta)
        thetadot = main_traj[i+1][2]/180*pi
        theta = main_traj[i+1][5]/180*pi
        xcmdot_post = main_traj[i+1][0]-h/2*thetadot*cos(theta)-w/2*thetadot*sin(theta)
        ycmdot_post = main_traj[i+1][1]+h/2*thetadot*sin(theta)+w/2*thetadot*cos(theta)
        fx = m*(xcmdot_post-xcmdot_pre)/dt         
        fy = m*(ycmdot_post-ycmdot_pre)/dt
        torque = I*(main_traj[i+1][2]-main_traj[i][2])/180*pi/dt
        fx_list.append(fx)
        fy_list.append(fy)
        torque_list.append(torque)
    # Actual forces/torques when contact starts
    # considering the large impact, we choose a little later when contact starts
    fx1 = fx_list[contact_idx+2]
    fy1 = fy_list[contact_idx+2]
    torque1 = torque_list[contact_idx+2]
    # Compute the desired force/torque (F = m(vref-0)/dt)
    fx_desired = m*mainblock_desired_state[5]/dt
    fy_desired = m*mainblock_desired_state[6]/dt
    torque_desired = I*mainblock_desired_state[7]/dt
    fx2 = fx_desired - fx1 
    fy2 = fy_desired - fy1 - m*g
    torque2 = torque_desired - torque1
    print("fx1, fy1, torque1, fx_desired, fy_desired, torque_desired {0}, {1}, {2}, {3}, {4}, {5}".format(fx1, fy1, torque1, fx_desired, fy_desired, torque_desired))
    # roughly compute (xcm+lx, ycm)
    if fx2 == 0 and fy2 == 0:
        p2 = None
    else:
        lx = torque2/sqrt(fx2**2+fy2**2)
        print()
        theta = mainblock_desired_state[4]/180*pi
        x2 = mainblock_desired_state[0] + mainblock_desired_state[2]/2 - lx*cos(theta) + mainblock_desired_state[3]/2*sin(theta)
        y2 = mainblock_desired_state[1] + mainblock_desired_state[3]/2 - lx*sin(theta) + mainblock_desired_state[3]/2*cos(theta)
        # we need the function to find the intersection 
        p2 = [x2,y2]
    return p2

def UpdateModelAfterFailure(guide_path_local, direction_end, block_type, block_state, s_ball_ini, u_pre, f_actual):
    # Update the mainblock's state after observations
    # Input : mainblock_type, previous_input, f_actual (x,y,dir) / env, xstart
    # Output : next_input or new model parameter

    # Model error term's update
    model_paramter = 0
    # ref : [x,y,dir]
    ref = np.array((guide_path_local[-1][0],guide_path_local[-1][1], direction_end))
    # y = (ball(5),ref(3),block type, block state)
    y = (s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],s_ball_ini[4], ref[0],ref[1],ref[2], block_type, block_state)  

    # Approximate Gradient descent 
    alpha = 0.3    
    jac = nd.Jacobian(fmodel)
    a = jac(u_pre, *y)
    b = a.T
    unew = u_pre - alpha*(2*np.matmul(b,f_actual)-2*np.matmul(b,ref))    
    return unew

def UpdateCostWeights():
    weights = [0,0,1]
    return weights

class Drawing:
    def __init__(self, xregion, yregion, xsize, ysize, s_grd_list, guide_path, guide_path_local, s_ball_ini):
        self.xregion = xregion
        self.yregion = yregion
        self.xsize = xsize
        self.ysize = ysize
        self.s_grd_list = s_grd_list
        self.guide_path = guide_path
        self.guide_path_local = guide_path_local
        self.s_ball_ini = s_ball_ini
        self.obj = []
    def drawlocalregion(self):
        # Input : xregion,yregion,xsize,ysize / s_grd_list,  s_ball_ini (x,y,vx,vy,r), guide_path_local
        # guide_path_local (from shortest path) : [[1,2],[2,3],...]
        '''
        ---------------------------------------------------------------------
        Draw the figure (We can make this be a function)
        ---------------------------------------------------------------------
        '''
        # Draw path planning figure
        fig1, self.ax1 = plt.subplots()
        self.ax1.axis('scaled')
        self.ax1.set(xlim = (0-1,500+1), ylim = (500+1,0-1))
        
        # Draw the red local region boundary
        local = patches.Rectangle((self.xregion,self.yregion),self.xsize,self.ysize,edgecolor='r', facecolor="none") 
        self.ax1.add_patch(local)

        # Prepare the PRM figure
        xpath = [self.guide_path[i][0] for i in range(len(self.guide_path))]
        ypath = [self.guide_path[i][1] for i in range(len(self.guide_path))]
        # Prepare the PRM figure
        xpath_local = [self.guide_path_local[i][0] for i in range(len(self.guide_path_local))]
        ypath_local = [self.guide_path_local[i][1] for i in range(len(self.guide_path_local))]
        # Draw the shortest path [[1,2],[2,3],...]
        self.ax1.plot(xpath, ypath, c = 'c', marker = 'o')
        self.ax1.plot(xpath_local,ypath_local, c = 'b', marker = 'o')

        # Draw the grounds and the ball / flag
        for s_grd in self.s_grd_list:
            ts = self.ax1.transData
            tr = trans.Affine2D().rotate_deg_around(s_grd[0]+s_grd[2]/2,s_grd[1]+s_grd[3]/2, s_grd[4])
            t = tr + ts # tr + ts (order is important)
            rect = patches.Rectangle((s_grd[0],s_grd[1]),s_grd[2],s_grd[3], edgecolor='k', facecolor="k", transform = t)
            self.ax1.add_patch(rect)
        ball = plt.Circle((self.s_ball_ini[0]+15,self.s_ball_ini[1]+15),self.s_ball_ini[4], facecolor = 'orange') # +15 means the ball's center
        self.ax1.add_artist(ball)
        self.ax1.plot()
        return self.ax1
    def updatelocalregion(self, new_local_path):
        self.guide_path_local = new_local_path
        xpath = [self.guide_path[i][0] for i in range(len(self.guide_path))]
        ypath = [self.guide_path[i][1] for i in range(len(self.guide_path))]
        xpath_local = [self.guide_path_local[i][0] for i in range(len(self.guide_path_local))]
        ypath_local = [self.guide_path_local[i][1] for i in range(len(self.guide_path_local))]
        self.ax1.plot(xpath, ypath, c = 'c', marker = 'o')
        self.ax1.plot(xpath_local,ypath_local, c = 'b', marker = 'o')
        return self.ax1
    def drawobject(self, obj, alpha_value = 1):
        if obj.__class__.__name__ == "woodBlock":
            # we have to differentiate block_type
            ts = self.ax1.transData
            tr = trans.Affine2D().rotate_deg_around(obj.x+obj.w/2,obj.y+obj.h/2, obj.rot)
            t = tr + ts # tr + ts (order is important)
            self.obj.append(patches.Rectangle((obj.x,obj.y),obj.w,obj.h, edgecolor='black', facecolor="y", transform = t, alpha = alpha_value))
            self.ax1.add_patch(self.obj[-1]) 
            self.ax1.plot()
        elif obj.__class__.__name__=="metalBlock":
            ts = self.ax1.transData
            tr = trans.Affine2D().rotate_deg_around(obj.x+obj.w/2,obj.y+obj.h/2, obj.rot)
            t = tr + ts # tr + ts (order is important)
            self.obj.append(patches.Rectangle((obj.x,obj.y),obj.w,obj.h, edgecolor='gray', facecolor="gray", transform = t, alpha = alpha_value))
            self.ax1.add_patch(self.obj[-1]) 
            self.ax1.plot()
        return self.obj
    def eraseobject(self):
        for i in range(len(self.obj)):
            del self.ax1.patches[-1]
        self.obj = []
        # print("obj list {}".format(self.obj))
        # [p.remove() for p in self.obj]

        #self.obj.remove()
        self.ax1.plot()
    def drawtraj(self, traj):
        # Draw the traj [vx,vy,w,x,y,rot]
        if len(traj) == 1:
            # when there is one element
            xpath = traj[0][3]
            ypath = traj[0][4]
        else:
            xpath = [traj[i][3] for i in range(len(traj))]
            ypath = [traj[i][4] for i in range(len(traj))]
        self.ax1.plot(xpath, ypath, c = 'm', marker = 's', markersize = 2)
        return self.ax1
    def drawpoint(self, pt):
        self.ax1.plot(pt[0],pt[1], c = 'r', marker = 'x')
        return self.ax1
    def drawball(self, ball_state):
        ball = plt.Circle((ball_state[0]+15,ball_state[1]+15),self.s_ball_ini[4], facecolor = 'orange') # +15 means the ball's center
        self.ax1.add_artist(ball)
        self.ax1.plot()
        return self.ax1


def ff(x,*y):    
    a = x[0]
    b = x[1]**2+y[0]*x[1]+y[1]
    r = np.array([a,b])
    return r

def MoveBlock2CloestBlock(block_state, block_type, env_local, env_type_list = []):
    # Check the distance between the optimal main block and the ground
    dist_min = 500
    dist_vector_min = [0,0]
    pt_min_moved = []
    for s_grd_local in env_local:
        dist_vector, dist, pt_moved = GetDistanceBlock2Block(block_state, s_grd_local, block_type, 'ground')
        if dist < dist_min:
            dist_min = dist
            dist_vector_min = dist_vector        
            pt_min_moved = pt_moved
    # Move the block to the point which is closest to the main block
    u_move = [block_state[0]+dist_vector_min[0], block_state[1]+dist_vector_min[1], block_state[4]]
    print("dist min{}".format(dist_min))
    return u_move, pt_min_moved

def CheckOverlap(state1, state2):
    # Search intersection
    pts1 = RotatePts(state1, 'rectangle')
    pts2 = RotatePts(state2, 'rectangle')
    intersect_pair = []
    bool_overlap = False

    for i in range(len(pts1)):
        bool_intersect, line_intersect, _ = CheckIntersectPolygon(pts2, pts1[i-1], pts1[i])
        if bool_intersect == True:
            bool_overlap = True
            for line in line_intersect:
                intersect_pair.append([line, [pts1[i-1], pts1[i]]])
    # Search inclusion of ALL points
    pt1_xmax = max([pt1[0] for pt1 in pts1])
    pt1_xmin = min([pt1[0] for pt1 in pts1])
    pt2_ymax = max([pt2[1] for pt2 in pts2])
    pt2_ymin = min([pt2[1] for pt2 in pts2])
    for pt1 in pts1:
        bool_inside2 = CheckInside(pts2, pt1)
        if bool_inside2 == True:
            bool_overlap == True
    for pt2 in pts2:
        bool_inside1 = CheckInside(pts1, pt2)
        if bool_inside1 == True:
            bool_overlap == True
    return bool_overlap, intersect_pair

def AdjustmentConstraint(main_state, env_local, support_state = []):
    # Check violation
    need_adjust = False
    if not support_state:
        pass
    else:
        b_overlap_support, intersect_pair_support = CheckOverlap(main_state, support_state)
        if b_overlap_support:
            need_adjust = True
    for env in env_local:
        b_overlap_env, intersect_pair_env = CheckOverlap(main_state, env)
        if b_overlap_env:
            need_adjust = True
    return need_adjust

def GetydistPt2grd(pt, ground_below):
    dist_min = 500
    pt_min = []
    grd_choice = []
    for grd in ground_below:
        pt_below, flag_intersect = GetFootVertical2block(pt, grd, 'ground')    
        if flag_intersect== True:
            dist_below = abs(pt_below[1] - pt[1])
            # Choose the closest one (other things are below that block)
            if dist_below < dist_min:
                dist_min = dist_below
                pt_min = pt_below
                grd_choice = grd
    return dist_min, pt_min, grd_choice


def Projection(p1, p2, ground_list):
    # return : the projected line of p1p2 onto the ground
    xmax = max(p1[0], p2[0])    
    xmin = min(p1[0], p2[0])
    ymin = min(p1[1], p2[1])
    ymax = 500
    projected_line1 = []
    projected_line2 = []
    print(xmax, ymax, xmin, ymin)
    grds_below = SortoutEnv(xmax, ymax, xmin, ymin, ground_list)
    if p1[0] > p2[0]: #p1 right
        pt_right = p1
        pt_left = p2
    else:
        pt_right = p2
        pt_left = p1

    if not grds_below:
        pts_list_from_left = []
        pts_list_from_right = [] 
    else:
        print("grd below {}".format(grds_below))        
        _, pt_below_r, grd_choice_r = GetydistPt2grd(pt_right, grds_below)
        _, pt_below_l, grd_choice_l = GetydistPt2grd(pt_left, grds_below)
        print("pt_below grd choice {}, {}, {}, {}".format(pt_below_l, pt_below_r, grd_choice_l, grd_choice_r))
        if not grd_choice_l and not grd_choice_r:
            pts_list_from_left = []
            pts_list_from_right = [] 
            # no intersect on left
        elif grd_choice_r and not grd_choice_l:
            vertices_r = RotatePts(grd_choice_r, 'ground')
            line_test_value_r = 100000
            for i in range(4):
                line_r = LineFrom2pts(vertices_r[i-1],vertices_r[i])
                a = line_r[0]
                b = line_r[1]
                c = line_r[2]
                if abs(a*pt_below_r[0]+b*pt_below_r[1]+c)<line_test_value_r:
                    if b == 0: # x = c
                        pass
                    elif a == 0: # y = c
                        line_test_value_r = abs(a*pt_below_r[0]+b*pt_below_r[1]+c)
                        left_pts_r = [vertices_r[i]]
                    elif -a/b >0: # positive angle                          
                        line_test_value_r = abs(a*pt_below_r[0]+b*pt_below_r[1]+c)
                        left_pts_r = [vertices_r[(i+1)%4], vertices_r[i]]
                    else: # negative angle                                  
                        line_test_value_r = abs(a*pt_below_r[0]+b*pt_below_r[1]+c)
                        left_pts_r = [vertices_r[i]]  
            pts_list_from_right = left_pts_r + [pt_below_r] 
            pts_list_from_left = []
        elif grd_choice_l and not grd_choice_r:
            vertices_l = RotatePts(grd_choice_l, 'ground')
            line_test_value_l = 100000
            for i in range(4):
                line_l = LineFrom2pts(vertices_l[i-1],vertices_l[i])
                a = line_l[0]
                b = line_l[1]
                c = line_l[2]
                if abs(a*pt_below_l[0]+b*pt_below_l[1]+c)<line_test_value_l:
                    if b == 0: # x = c
                        pass
                    elif a == 0: # y = c
                        line_test_value_l = abs(a*pt_below_l[0]+b*pt_below_l[1]+c)
                        right_pts_l = [vertices_l[i-1]]
                    elif -a/b>0:
                        right_pts_l = [vertices_l[i-1]]                    
                        line_test_value_l = abs(a*pt_below_l[0]+b*pt_below_l[1]+c)
                    else:
                        right_pts_l = [vertices_l[i-1], vertices_l[(i-2)%4]]                    
                        line_test_value_l = abs(a*pt_below_l[0]+b*pt_below_l[1]+c)
            pts_list_from_right = []
            pts_list_from_left = [pt_below_l] + right_pts_l 
        else:
            vertices_r = RotatePts(grd_choice_r, 'ground')
            vertices_l = RotatePts(grd_choice_l, 'ground')
            bool_overlap, intersect_pairs = CheckOverlap(grd_choice_l, grd_choice_r)
            # left, right are projected on the same ground
            if grd_choice_l == grd_choice_r:
                pts_list_from_left = [pt_below_l]
                pts_list_from_right = [pt_below_r]
            else:
                line_test_value_r = 100000
                line_test_value_l = 100000
                # To get the other points between right pt and left pt
                for i in range(4):
                    line_r = LineFrom2pts(vertices_r[i-1],vertices_r[i])
                    a = line_r[0]
                    b = line_r[1]
                    c = line_r[2]
                    if abs(a*pt_below_r[0]+b*pt_below_r[1]+c)<line_test_value_r:
                        if b == 0: # x = c
                            pass
                        elif a == 0: # y = c
                            line_test_value_r = abs(a*pt_below_r[0]+b*pt_below_r[1]+c)
                            left_pts_r = [vertices_r[i]]
                        elif -a/b >0: # positive angle                          
                            line_test_value_r = abs(a*pt_below_r[0]+b*pt_below_r[1]+c)
                            left_pts_r = [vertices_r[(i+1)%4], vertices_r[i]]
                        else: # negative angle                                  
                            line_test_value_r = abs(a*pt_below_r[0]+b*pt_below_r[1]+c)
                            left_pts_r = [vertices_r[i]]      
                for i in range(4):
                    line_l = LineFrom2pts(vertices_l[i-1],vertices_l[i])
                    a = line_l[0]
                    b = line_l[1]
                    c = line_l[2]
                    if abs(a*pt_below_l[0]+b*pt_below_l[1]+c)<line_test_value_l:
                        if b == 0: # x = c
                            pass
                        elif a == 0: # y = c
                            line_test_value_l = abs(a*pt_below_l[0]+b*pt_below_l[1]+c)
                            right_pts_l = [vertices_l[i-1]]
                        elif -a/b>0:
                            right_pts_l = [vertices_l[i-1]]                    
                            line_test_value_l = abs(a*pt_below_l[0]+b*pt_below_l[1]+c)
                        else:
                            right_pts_l = [vertices_l[i-1], vertices_l[(i-2)%4]]                    
                            line_test_value_l = abs(a*pt_below_l[0]+b*pt_below_l[1]+c)
                if bool_overlap == True:
                    intersect_pts = []
                    for intersect_pair in intersect_pairs:
                        intersect_pt_temp = GetIntersectPt(intersect_pair[0][0], intersect_pair[0][1], intersect_pair[1][0], intersect_pair[1][1])
                        intersect_pts.append(intersect_pt_temp)
                    combine_r = left_pts_r + intersect_pts
                    combine_l = right_pts_l + intersect_pts
                    left_pts_r = combine_r
                    right_pts_l = combine_l
                print("left pts r {} {} \n".format(left_pts_r, right_pts_l))
                # we can add vertices of the block inside

                # Decide the order of pts
                pts_list_from_left = [pt_below_l]
                pts_list_from_right = [pt_below_r]
                # Check y value from the above
                sequence = right_pts_l + left_pts_r        
                sequence_array = np.array(sequence)
                # sort by the order of y (above is first)
                sequence_yorder_array = sequence_array[np.argsort(sequence_array[:,1])]
                sequence_yorder = sequence_yorder_array.tolist()
                for pt in sequence_yorder:
                    if pt[0] < pts_list_from_right[0][0] and pt[0] > pts_list_from_left[-1][0]: # if inside                
                        if pt in right_pts_l and pt not in left_pts_r: # check where it comes from
                            pts_list_from_left.append(pt) #[~,~,pt]
                        elif pt in left_pts_r and pt not in right_pts_l:
                            pts_list_from_right.insert(0, pt) #[pt, ~, ~]
                        else: # belong to both
                            pts_list_from_left.append(pt) #[~,~,pt]
                            pts_list_from_right.insert(0, pt) #[pt, ~, ~]
                    else: # outside
                        pass
                if pts_list_from_left[-1][1]>pts_list_from_right[0][1]:
                    # right part is above
                    pt_left_foot, intersect = GetFootVertical2block(pts_list_from_right[0], grd_choice_l, 'ground')
                    if intersect == True:
                        pts_list_from_left.append(pt_left_foot)
                    else:
                        pass
                else:
                    pt_right_foot, intersect = GetFootVertical2block(pts_list_from_left[-1], grd_choice_r, 'ground')
                    if intersect == True:
                        pts_list_from_right.insert(0, pt_right_foot)
                    else:
                        pass          
    # grd_choice_l, grd_choice_r can be returned
    return pts_list_from_left, pts_list_from_right 


def Move2ClosePt(u_actual, block_state, block_type, s_grd_list):
    w_block = block_state[2]
    h_block = block_state[3]
    block_state_actual = [u_actual[0], u_actual[1], w_block, h_block, u_actual[2]]
    # Decide the region below the main block (roughly we can use [x,y,x+w,ymax])
    pts_actual = RotatePts(block_state_actual, block_type)
    x_actual_list = [pts_actual[i][0] for i in range(len(pts_actual))]
    y_actual_list = [pts_actual[i][1] for i in range(len(pts_actual))]
    x_actual_min = min(x_actual_list)
    y_actual_min = min(y_actual_list)
    x_actual_max = max(x_actual_list)
    y_actual_max = 500

    # 5) Prioritize the remaining blocks and place one roughly based on only geometry

    # Check which grounds exist below the mainblock now among local_env
    grounds_below = SortoutEnv(x_actual_max, y_actual_max, x_actual_min, y_actual_min, s_grd_list)
    print("ground below {}".format(grounds_below))
    
    # Check the other point's shortest distance upto the ground
    dist_min = 500
    dist_min_left = 500
    dist_min_right = 500
    # for rectangle
    pt_left = pts_actual[3]
    pt_right = pts_actual[0]
    pt_min = []
    pt_min_right = []
    pt_min_left = []
    grd_choice_right = []
    grd_choice_left = []

    dist_min_left, pt_min_left, grd_choice_left = GetydistPt2grd(pt_left, grounds_below)
    dist_min_right, pt_min_right, grd_choice_right = GetydistPt2grd(pt_right, grounds_below)

    print("pt_min_right, pt_min_left, dist_min_right, dist_min_left : {}, {}, {}, {}".format(pt_min_right, pt_min_left,  dist_min_right, dist_min_left))
    
    # Search neighborhood to check if we can move a little bit
    pt_move_left = []
    pt_move_right = []
    move_standard_low = 10
    move_standard_high = 50
    num_search = 4   
    
    if dist_min_left > move_standard_low and dist_min_left < move_standard_high:
        for i in range(num_search):
            pt_search = [pt_left[0]+(i+1)*dist_min_left/num_search, pt_left[1]]
            dist_min_search, pt_min_search, _ = GetydistPt2grd(pt_search, grounds_below) #env_local is correct                   
            if dist_min_search < dist_min_left:
                pt_move_left = pt_min_search
                dist_min_left = dist_min_search
    if dist_min_right > move_standard_low and dist_min_right < move_standard_high:
        for i in range(num_search):
            pt_search = [pt_right[0]+(i+1)*dist_min_right/num_search, pt_right[1]]
            dist_min_search, pt_min_search, _ = GetydistPt2grd(pt_search, grounds_below)
            print("dist_min_search, pt_min_search {} {}".format(dist_min_search, pt_min_search ))
            if dist_min_search < dist_min_right:
                pt_move_right = pt_min_search
                dist_min_right = dist_min_search

    # Move the main block 
    if not pt_move_left and not pt_move_right:
        xmove = 0
        ymove = 0
    elif len(pt_move_left)>0 and not pt_move_right:
        xmove = pt_move_left[0] - pt_left[0]
        ymove = pt_move_left[1] - pt_left[1]
        dist_min_left = 0
    elif not pt_move_left and len(pt_move_right)>0:
        xmove = pt_move_right[0] - pt_right[0]
        ymove = pt_move_right[1] - pt_right[1]
        dist_min_right = 0
    else:
        if dist_min_right > dist_min_left:
            xmove = pt_move_left[0] - pt_left[0]
            ymove = pt_move_left[1] - pt_left[1]
            dist_min_left = 0
        else:
            xmove = pt_move_right[0] - pt_right[0]
            ymove = pt_move_right[1] - pt_right[1]
            dist_min_right = 0

    u_move = [u_actual[0] + xmove, u_actual[1] + ymove, u_actual[2]] 
    print("u_move and xmove, ymove")
    print(u_move, xmove, ymove)

    # Update
    block_state_move = [u_move[0], u_move[1], w_block, h_block, u_move[2]]
    pts_move = RotatePts(block_state_move, block_type)
    pt_left = pts_move[3]
    pt_right = pts_move[0]

    # Check pt_min again
    dist_min_left, pt_min_left, grd_choice_left = GetydistPt2grd(pt_left, grounds_below)
    dist_min_right, pt_min_right, grd_choice_right = GetydistPt2grd(pt_right, grounds_below)

    return u_move, [pt_min_left, pt_min_right], [dist_min_left, dist_min_right], [grd_choice_left, grd_choice_right]

def PrioritizeBlocks(dist_min, pt_min, num_movable, movable_ID, ID_dict, ID_state_matching):
    idx_order_from_closest_to_farthest = []
    gap_list = []
    if dist_min < 25 or not pt_min: # no need for supporting blocks
        pass
    else:
        for pick_support_idx in range(num_movable):
            if pick_support_idx == pick_main_idx:
                pass
            else:
                # Pick supporting blocks / we can add some priorities    
                support_id = movable_ID[pick_support_idx]
                support_block_type = ID_dict[support_id] # 'metalrectangle'
                support_block_state = ID_state_matching[support_id] # [x,y,w,h,rot]
                w_temp = support_block_state[2]
                h_temp = support_block_state[3]
                gap_temp = min(abs(dist_min-w_temp), abs(dist_min-h_temp))
                if not gap_list:
                    gap_list.append(gap_temp)
                    idx_order_from_closest_to_farthest.append(pick_support_idx)
                else:
                    for j in range(len(gap_list)):
                        if gap_list[j] >= gap_temp:
                            gap_list.insert(j, gap_temp)
                            idx_order_from_closest_to_farthest.insert(j, pick_support_idx)
    print(idx_order_from_closest_to_farthest)
    return idx_order_from_closest_to_farthest, gap_list

def DecideSupportingState(idx_order_from_closest_to_farthest, pt_min, dist_min, grd_choice, movable_ID, ID_dict, ID_state_matching, u_move, w_block, h_block, env_local):
    # Return : x_support, y_suuport, w_support, h_support, theta_support, u_move, env_local_temp
    if not idx_order_from_closest_to_farthest:
        support_idx = None
        support_id = None
        support_type = None
    else:
        # rough guess of configuration of supporting blocks
        support_idx = idx_order_from_closest_to_farthest[0]
        support_id = movable_ID[support_idx]
        support_type = ID_dict[support_id] # 'metalrectangle'
        support_state = ID_state_matching[support_id] # [x,y,w,h,rot]
        w_support = support_state[2]
        h_support = support_state[3]
        if pt_min[0] < block_state_desired[0]+w_block/2: # move right, left point is left
            x_support = pt_min[0]
            y_support = pt_min[1]-h_support
        else:
            x_support = pt_min[0]-w_support/2
            y_support = pt_min[1]-h_support
        theta_support = grd_choice[4] # same with the angle of underlying ground
        if abs(w_support-dist_min) < abs(h_support-dist_min):
            theta_support += 90
            x_support -= w_support/2
            y_support -= w_support/2-h_support/2

    # Check feasibility (between supporting and main)
    if not idx_order_from_closest_to_farthest:
        pass
    else:
        u_support = [x_support, y_support, w_support, h_support, theta_support]                    
        main_state = [u_move[0], u_move[1], w_block, h_block, u_move[2]]
        bool_overlap, _ = CheckOverlap(main_state, u_support)
        while bool_overlap == True:                        
            u_move[1] -= 10
            # or we can do rotation
            main_state = [u_move[0], u_move[1], w_block, h_block, u_move[2]]
            bool_overlap, _ = CheckOverlap(main_state, u_support)
    # If we have supporting blocks, we also consider this when we check COM's stability
    if not idx_order_from_closest_to_farthest:
        env_local_temp = env_local 
        state_support_temp = []
    else:
        state_support_temp = [x_support, y_support, w_support, h_support, theta_support]
        env_local_temp = env_local  + [state_support_temp]

    
    print(state_support_temp)
    return state_support_temp, u_move, env_local_temp


if __name__=="__main__":

    # Test for a local region
    level_select = 4
    # 0) Prerequisite : guide path
    # guide_path = [[20,135],[50,155],[110,150],[180,170],[230,185],[280,175],[310,185],[340,195],[400,200]]
    #guide_path = [[450, 120], [443, 158], [411, 167], [379, 192], [350, 205], [315, 177], [276, 202], [241, 195], [204, 186], [166, 178], [135, 196], [104, 209], [78, 207], [61, 202], [34, 195]]
    guide_path = [[75, 100], [99, 120], [120, 126], [142, 136], [178, 144], [220, 141], [230, 153], [266, 133], [290, 169], [331, 153], [338, 154], [383, 195], [424, 217], [440, 240], [439, 254], [419, 285]]
    #guide_path = [[210, 100], [241, 119], [284, 139], [304, 129], [348, 137], [372, 144], [391, 151], [435, 153], [476, 167], [446, 210], [407, 223], [389, 227], [377, 260], [345, 276], [310, 274], [276, 263], [255, 283], [243, 287], [209, 294], [176, 288], [159, 292], [139, 293], [117, 300], [86, 302], [59, 292], [34, 285]]
    #guide_path = [[25, 100], [38, 119], [79, 132], [108, 154], [126, 142], [167, 137], [195, 134], [230, 147], [239, 150], [275, 154], [295, 179], [327, 202], [356, 212], [384, 222], [414, 234], [417, 253], [419, 275]]
    state_input = [] # decided ahead in previous local regions
    #state_input.append(["AO6G", 0,0,0])
    
    # 1) Predict the ball's traj, load a traj. from a log file
    id_grd, s_grd_list, s_total, id_total, n_total, movable_ID, ID_dict, ID_state_matching = parsing_objects(level_select)
    state_input.append(["{}".format(movable_ID[0]), 0,0,0])
    run_simulation(level_select, movable_ID, ID_dict, state_input)
    state_input = []
    traj_list, trace_time, collision, n_obj, bool_success, ID_list, _, _ = logging_trajectory("bb", level_select)       
    id_ball = id_total[0]
    s_ball = s_total[0] # [x,y,w,h,rot], we use only w and h here
    # check the order of the ID in the ID_list
    ball_idx = 0
    while id_ball != ID_list[ball_idx] and ball_idx < len(ID_list):
        ball_idx = ball_idx+1
    ball_traj_init = traj_list[ball_idx] # [vx,vy,wrot,x,y,theta] 6 by N
    
    # 2) Choose a local region and cut-off the guide path locally
    x_predicted_traj = [ball_traj_init[j][3] for j in range(len(ball_traj_init))]
    y_predicted_traj = [ball_traj_init[j][4] for j in range(len(ball_traj_init))]

    guide_path_local, direction_end, x_local_max, y_local_max, idx_pick_start, idx_pick_end = SelectLocalRegion(guide_path, x_predicted_traj, y_predicted_traj, s_grd_list)
    xregion = min(guide_path_local[0][0],guide_path_local[-1][0], x_predicted_traj[idx_pick_start], x_predicted_traj[idx_pick_end])
    yregion = min(guide_path_local[0][1],guide_path_local[-1][1], y_predicted_traj[idx_pick_start], y_predicted_traj[idx_pick_end])
    #xregion = min(x_predicted_traj[idx_pick_start], x_predicted_traj[idx_pick_end])
    #yregion = min(y_predicted_traj[idx_pick_start], y_predicted_traj[idx_pick_end])
    xsize = x_local_max-xregion
    ysize = y_local_max-yregion
    local_region = [x_local_max, y_local_max, xregion, yregion]
    print("local_region {}".format(local_region))
    # s_ball_ini = [x,y,vx,vy,r] : so we have to change the order because log file has vx,vy first
    s_ball_ini = [ball_traj_init[idx_pick_start][3],ball_traj_init[idx_pick_start][4],ball_traj_init[idx_pick_start][0],ball_traj_init[idx_pick_start][1],15]
    # sort out the grounds' states from the total s_grd_list
    env_local = SortoutEnv(x_local_max, y_local_max, xregion, yregion, s_grd_list)
    # get the types and states of envs
    env_left = [500,0,0,0,0]
    env_right = [0,0,0,0,0]
    for env_element in env_local:
        if env_element[0] < env_left[0]:
            env_left = env_element
    for env_element in env_local:
        if env_element[0] > env_right[0]:
            env_right = env_element
    if s_ball_ini[2] >= 0: # go to the right
        env_type_pre = "air"
        env_state_pre = env_left
        env_type_post = "air"
        env_state_post = env_right
    else:
        env_type_pre = "air"
        env_state_pre = env_right
        env_type_post = "air"
        env_state_post = env_left
    print("envlocal:{}".format(env_local))
    print(env_left, env_right)

    display = Drawing(xregion, yregion, xsize, ysize, s_grd_list, guide_path, guide_path_local, s_ball_ini)
    display.drawlocalregion()
    plt.show(block=False)
    plt.pause(2)    

    '''
    ----------------------------------------------------------------------------------------------
    This is the start of the loop for a local region (the largest loop for picking a new main block)
    ** 1 st loop
    ----------------------------------------------------------------------------------------------
    '''
    count_1st_loop = 0
    error_min = 50000
    err_criterion_ball = 5000
    # for erasing figure
    obj_main_pre = []
    obj_support_pre = []
    # While the local region fails, do a loop
    num_movable = len(movable_ID)
    pick_main_idx = 0
    while  bool_success == False and error_min > err_criterion_ball and count_1st_loop < num_movable:
        count_1st_loop += 1
        if count_1st_loop != 1:
            pick_main_idx += 1
        else:
            pass
    
        # 3) Pick a main block / we can add some priorities    
        input_id = movable_ID[pick_main_idx]
        block_type = ID_dict[input_id] # 'metalrectangle'
        block_state = ID_state_matching[input_id] # [x,y,w,h,rot]

        print("guide_path_local {}".format(guide_path_local))

        '''
        -------------------------------------------------------
        Small loop for solving optmization problem repeatedly
        ** 2nd, 3rd loop
        -------------------------------------------------------
        '''
        count_2nd_loop = 0
        #unew = UpdateModelAfterFailure(guide_path_local, direction_end, block_type, block_state, s_ball_ini, u_optimal, f_actual)

        '''
        -------------------------------------------------------
        Small loop for solving optmization problem repeatedly with adaptive weights
        ** 3rd loop
        -------------------------------------------------------
        '''
          
        count_3rd_loop = 0
        traj_sorted_init, idx_sorted = SortedTrajInLocal(ball_traj_init, local_region)
        #print("traj_sorted_init {}".format(traj_sorted_init))

        # 4) Solve the optimization problem to obtain the required state of the main block
        while bool_success == False and error_min > err_criterion_ball and count_3rd_loop < 2:
            count_3rd_loop += 1
            # 4)-0. Adpat the local region size after observations (reduce the size)
            # update guide path local
            if count_3rd_loop != 1:
                guide_path_local_update = [guide_path_local[i] for i in range(len(guide_path_local)-count_3rd_loop+1)]
                #del guide_path_local[-1]
                #print(guide_path_local)
                #u_optimal = FindOptimalInput(guide_path_local, direction_end, block_type, block_state, s_ball_ini, env_local)
                display.updatelocalregion(guide_path_local_update)                        
                adaptive_length = 25
            else:
                # first iteration
                guide_path_local_update = [guide_path_local[i] for i in range(len(guide_path_local))] # to avoid deep copy
                adaptive_length = 25       

            # 4)-1. get the main block's optimal state with fixed x1 (we will update this later)
            w_block = block_state[2]
            h_block = block_state[3]

            adaptive_length_resolution = 20
            adaptive_length_grid = range(0,int(xsize/3),adaptive_length_resolution)
            umain_length_fobj_tuple_list_extend = []
            f_obj_min = 10000000

            print("time start : {}".format(time.time()))
            #print(traj_sorted_init)
            for adaptive_length in adaptive_length_grid:
                if abs(s_ball_ini[2]) < 0.5:
                    # vx = 0, fall downward
                    state_ball = MakeF1withTraj(traj_sorted_init, adaptive_length)
                    # state_ball = BallinAirValue(s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],0,adaptive_length)
                else:
                    state_ball = MakeF1withTraj(traj_sorted_init, adaptive_length)
                    # state_ball = BallinAirValue(s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],adaptive_length)
                
                # Update s_ball_ini for main block
                s_ball_mid_temp = [state_ball[0], state_ball[1], state_ball[2], state_ball[3], s_ball_ini[4]]
                print("state ball {}".format(state_ball))
                #display.drawball(state_ball)

                # Find the optimal solution for a main block
                # 1) grid for u (x1 is fixed before)
                # grid 
                
                u_optimal_temp, f_obj_min_temp, umain_fobj_tuple_list = FindOptimalInputGrid(guide_path_local_update, direction_end, block_type, block_state, s_ball_mid_temp, env_local)
                for umain_fobj_tuple in umain_fobj_tuple_list:
                    umain_length_fobj_tuple = (umain_fobj_tuple[0], adaptive_length, umain_fobj_tuple[1])
                    umain_length_fobj_tuple_list_extend.append(umain_length_fobj_tuple)
                if f_obj_min_temp <= f_obj_min:
                    f_obj_min = f_obj_min_temp
                    u_optimal = u_optimal_temp
                    s_ball_mid = s_ball_mid_temp

            # sort the elements in an asending order
            def get_fobj(elem):
                return elem[2]
            umain_length_fobj_tuple_list_extend.sort(key=get_fobj)
            print("time end : {}".format(time.time()))
            print(umain_length_fobj_tuple_list_extend)

            #u_optimal = FindOptimalInput(guide_path_local_update, direction_end, block_type, block_state, s_ball_ini, env_local)
            #u_optimal = FindOptimalInputConstrained(guide_path_local_update, direction_end, block_type, block_state, s_ball_ini, env_type_pre, env_state_pre, env_type_post, env_state_post, env_local, local_region)
            

            # after f1, need to get the state just before entering fu
            # We can use the actual data and get the y value at x(u_optimal[5])
            # if env_type_pre == "ground":
            #     state_ball, _ = Ball2LineValue(s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],s_ball_ini[4], u_optimal[5]/cos(env_state_pre[4]/180*pi), env_state_pre[4], 0, 0, 0)
            # else: #elif y[10] == "air":
            #     if abs(s_ball_ini[2]) < 0.5:
            #         state_ball = BallinAirValue(s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],0,u_optimal[5])
            #     else:
            #         state_ball = BallinAirValue(s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],u_optimal[5])
            # s_ball_mid = [state_ball[0], state_ball[1], state_ball[2], state_ball[3], s_ball_ini[4]]
            # u_actual, vel_desired = ConvertUopt2Ureal(u_optimal, block_type, w_block, h_block, s_ball_mid)

            u_actual, vel_desired = ConvertUopt2Ureal(u_optimal, block_type, w_block, h_block, s_ball_mid)
            block_state_desired = [u_actual[0], u_actual[1], w_block, h_block, u_actual[2]]


            # 4)-2. Move the block to the point which is closest to the main block (only when the block is wood)
            if block_type[0:4] == "wood":
                # move the block a little bit to closest block
                #u_move, pt_min_moved = MoveBlock2CloestBlock(block_state_desired, block_type, env_local, env_type_list = [])
                
                u_move, pt_min_lr, dist_min_lr, grd_choice_lr = Move2ClosePt(u_actual, block_state, block_type, s_grd_list)
                pt_min_left = pt_min_lr[0]
                pt_min_right = pt_min_lr[1]
                dist_min_left = dist_min_lr[0]
                dist_min_right = dist_min_lr[1]
                grd_choice_left = grd_choice_lr[0]
                grd_choice_right = grd_choice_lr[1]
                block_state_move = [u_move[0], u_move[1], w_block, h_block, u_move[2]]
                pts_move = RotatePts(block_state_move, 'ground')
                pt_left = pts_move[3]
                pt_right = pts_move[0]
                # Compare the distance with some sizes of block
                # idx_order_from_closest_to_farthest : priority order

                idx_order_from_closest_to_farthest_left, gap_list_left = PrioritizeBlocks(dist_min_left, pt_min_left, num_movable, movable_ID, ID_dict, ID_state_matching)
                idx_order_from_closest_to_farthest_right, gap_list_right = PrioritizeBlocks(dist_min_right, pt_min_right, num_movable, movable_ID, ID_dict, ID_state_matching)
        

                # 6) Decide the position of supporting blocks

                # mainblock_desired_state(x,y,w,h,rot,vx,vy,w_rot)
                mainblock_desired_state = [u_move[0],u_move[1],w_block,h_block,u_move[2],vel_desired[0],vel_desired[1],vel_desired[2]]
                state_support_temp, u_move, env_local_temp = DecideSupportingState(idx_order_from_closest_to_farthest_left, pt_min_left, dist_min_left, grd_choice_left, movable_ID, ID_dict, ID_state_matching, u_move, w_block, h_block, env_local)
                state_support_temp, u_move, env_local_temp = DecideSupportingState(idx_order_from_closest_to_farthest_right, pt_min_right, dist_min_right, grd_choice_left, movable_ID, ID_dict, ID_state_matching, u_move, w_block, h_block, env_local)
                
                if not state_support_temp:
                    pass
                else:
                    x_support = state_support_temp[0]
                    y_support = state_support_temp[1]
                    w_support = state_support_temp[2]
                    h_support = state_support_temp[3]
                    theta_support = state_support_temp[4]


                # General thing with projection
                interval_num = 5
                grid4projection = []
                pts_list_from_left_concatenate = []
                pts_list_from_right_concatenate = []
                for i in range(interval_num+1):
                    x_temp = (pt_left[0]*(interval_num-i)+i*pt_right[0])/interval_num
                    y_temp = (pt_left[1]*(interval_num-i)+i*pt_right[1])/interval_num
                    grid4projection.append([x_temp, y_temp])
                for i in range(interval_num):
                    pts_list_from_left_temp, pts_list_from_right_temp = Projection(grid4projection[i],grid4projection[i+1], env_local_temp)
                    print("pts_list_from_left_temp right temp {} {}".format(pts_list_from_left_temp, pts_list_from_right_temp))
                    pts_list_from_left_concatenate.extend(pts_list_from_left_temp)
                    pts_list_from_left_concatenate.extend(pts_list_from_right_temp)

                # for one interval
                pts_list_from_left, pts_list_from_right = Projection(pt_left, pt_right, env_local_temp)
                #pts_list_from_left = pts_list_from_left_concatenate
                #pts_list_from_right = pts_list_from_left_concatenate
                print("pts_list_from_left and right {}, {}".format(pts_list_from_left, pts_list_from_right))
                print(state_support_temp)

                # For stability, COM should be between the closest 2 points. If not, we have to move it
                # Move the block so that COM has same x as the closest point
                dist_left = []
                dist_right = []
                if not pts_list_from_right and not pts_list_from_left:
                    # need to move
                    pass
                elif not pts_list_from_right and pts_list_from_left:
                    for pt_l in pts_list_from_left:
                        dist_left.append(GetDistancePt2Line(pt_l, pt_left, pt_right))
                elif pts_list_from_right and not pts_list_from_left:
                    for pt_r in pts_list_from_right:
                        dist_right.append(GetDistancePt2Line(pt_r, pt_left, pt_right))
                else:
                    for pt_l in pts_list_from_left:
                        dist_left.append(GetDistancePt2Line(pt_l, pt_left, pt_right))
                    for pt_r in pts_list_from_right:
                        dist_right.append(GetDistancePt2Line(pt_r, pt_left, pt_right))

                pts_combine = pts_list_from_left + pts_list_from_right        
                dist_combine = dist_left + dist_right
                dist_combine_array = np.array(dist_combine)
                idx_shortest_order= np.argsort(dist_combine_array)  
                criteria = 20
                pts_below_close = []
                for i in idx_shortest_order:
                    if dist_combine[i] < criteria:
                        pt_below_min_temp = pts_combine[i]
                        pts_below_close.append(pt_below_min_temp)
                # If there are no pts who has smaller distance than 20        
                if len(pts_below_close) < 2:
                    pts_below_close.append(pts_combine[0])
                    pts_below_close.append(pts_combine[1])

                print("pts below close dist comb {} {}".format(pts_below_close, dist_combine_array))
                pts_below_close_array = np.array(pts_below_close)
                array_temp = pts_below_close_array[np.argsort(pts_below_close_array[:,0])]
                pt_below_min1= array_temp[0] # most left
                pt_below_min2 = array_temp[-1] # most right
                # COM's position
                com = [u_move[0]+w_block/2, u_move[1]+h_block/2]
                if com[0] < max(pt_below_min1[0], pt_below_min2[0]) and com[0]>min(pt_below_min1[0], pt_below_min2[0]):
                    # inside -> stable / we can guess it maintains
                    xmove = 0
                elif com[0] < min(pt_below_min1[0], pt_below_min2[0]):
                    # outside -> we have to move it
                    xmove = min(pt_below_min1[0], pt_below_min2[0])-com[0]+10
                else:
                    xmove = max(pt_below_min1[0], pt_below_min2[0])-com[0]-10

                #u_move[0] += xmove
                print("distcombine, xmove u_move[0] com[0]: {} {} {} {}".format(dist_combine, xmove, u_move[0], com[0]))

            '''
            -------------------------------------------------------
            Smallest loop for assinging the supporting blocks repeatedly
            ** 4rd loop
            -------------------------------------------------------
            '''
            if block_type[0:4] != "wood":
                # Adjustment of constraints

                # Input the main block.                      
                state_input_localregion = []   
                state_input_localregion.append([input_id, u_actual[0], u_actual[1], u_actual[2]])
                # draw the figure with blocks
                if block_type == "metalrectangle" or block_type == "metalrtriangle":
                    metalblock = metalBlock(u_actual[0],u_actual[1],w_block,h_block,u_actual[2])
                    alpha = 1.1-(count_1st_loop + count_2nd_loop + count_3rd_loop)*0.05
                    display.drawobject(metalblock, alpha)                
                plt.show(block=False)
                plt.pause(5)
                state_input.extend(state_input_localregion)
                run_simulation(level_select, movable_ID, ID_dict, state_input)
                for i in range(len(state_input_localregion)):
                    del state_input[-1]
                # Observe again about the main, ball, support blocks trajectory and check when the contact between blocks is active (contact_idx)
                traj_list, trace_time, collision, n_obj, bool_success, ID_list, _, _ = logging_trajectory("bb", level_select)   
                # traj : [vx,vy,w,x,y,rot]
                # check the order of the ID in the ID_list 
                ball_traj = traj_list[ball_idx]  
                mainblock_traj = traj_list[pick_main_idx+1] # in log file, we include ball, so we need to add 1 

                # 7) Check the error between guide_path_local, direction_end and mainblock_traj at x_local  
                # ref : [x,y,dir] / state_ball : [x,y,vx,vy]
                ref = [guide_path_local_update[-1][0],guide_path_local_update[-1][1], direction_end]
                traj_sorted, idx_sorted = SortedTrajInLocal(ball_traj, local_region)
                # get the minimum of the error between guide path and each element of the entire trajectory
                for i in range(len(traj_sorted)):
                    state_ball_current = [traj_sorted[i][3], traj_sorted[i][4], traj_sorted[i][0], traj_sorted[i][1]]
                    error, error_vector = GetError(ref, state_ball_current)  
                    if error <= error_min:
                        error_min = error
                        index_min = i                     
                # deviation between the actual traj. of main block and desired traj. of main block
                # desired_traj_main [vx,vy,w,x,y,rot]
                display.drawtraj(traj_sorted)
                plt.show(block=False)
                plt.pause(3)
                print("error : {}".format(error_min))
                print("iteration 1st loop, 2nd loop, 3rd loop: {0}, {1}, {2}".format(count_1st_loop, count_2nd_loop, count_3rd_loop))
            else:
                # if type is wood
                err_main_criterion = 2000
                err_main = 50000
                flag_supportings = True
                count_4th_loop = 0
                while bool_success == False and error_min > err_criterion_ball and err_main > err_main_criterion and flag_supportings and count_4th_loop <2 and  block_type[0:4] == "wood":
                    count_4th_loop += 1
                    print("iteration 1st loop, 2nd loop, 3rd loop, 4th loop : {0}, {1}, {2}, {3}".format(count_1st_loop, count_2nd_loop, count_3rd_loop, count_4th_loop))
                    if count_4th_loop == 1:
                        if not idx_order_from_closest_to_farthest:
                            # we have to change the optimization solution
                            flag_supportings = False
                        pass
                    else:
                        # 8)-0. From required force or error_main_array, we can update the supporting blocks    
                        if not idx_order_from_closest_to_farthest:
                            # we have to change the optimization solution
                            flag_supportings = False
                        else:
                            # rough guess of configuration of supporting blocks
                            support_idx = idx_order_from_closest_to_farthest[0]
                            support_id = movable_ID[support_idx]
                            support_type = ID_dict[support_id] # 'metalrectangle'
                            support_state = ID_state_matching[support_id] # [x,y,w,h,rot]
                            w_support = support_state[2]
                            h_support = support_state[3]                        

                            if pt_min[0] > block_state_desired[0]+w_block/2: # move right, left point is left
                                x_support = x_support + error_main_array[3]
                                y_support = y_support + error_main_array[4]
                            else:
                                x_support = pt_min[0] + error_main_array[3]
                                y_support = pt_min[1] + error_main_array[4]
                            theta_support = grd_choice[4] # same with the angle of underlying ground
                            if abs(w_support-dist_min) < abs(h_support-dist_min):
                                theta_support += 90

                    if flag_supportings == False:
                        pass
                    else:
                        print("ggg")

                        u_support = [x_support, y_support, w_support, h_support, theta_support]

                        # Check feasibility (between suppoting and main)
                        main_state = [u_move[0], u_move[1], w_block, h_block, u_move[2]]
                        bool_overlap, _ = CheckOverlap(main_state, u_support)
                        while bool_overlap == True:
                            u_move[1] -= 10
                            main_state = [u_move[0], u_move[1], w_block, h_block, u_move[2]]
                            bool_overlap, _ = CheckOverlap(main_state, u_support)
     

                        # Get a new feasible solution among the previous feasible set

                        # print("check time start {}".format(time.time()))
                        # for umain_length_fobj_tuple in umain_length_fobj_tuple_list_extend:
                        #     # umain_length_fobj_tuple = (u_main, adaptive_length, f_obj)
                        #     u_main = umain_length_fobj_tuple[0]  # l, theta form 
                        #     adaptive_length = umain_length_fobj_tuple[1]
                        #     if abs(s_ball_ini[2]) < 0.5:
                        #         # vx = 0, fall downward
                        #         state_ball = MakeF1withTraj(traj_sorted_init, adaptive_length)
                        #     else:
                        #         state_ball = MakeF1withTraj(traj_sorted_init, adaptive_length)                    
                        #     s_ball_mid_temp = [state_ball[0], state_ball[1], state_ball[2], state_ball[3], s_ball_ini[4]]
                        #     u_move, vel_desired = ConvertUopt2Ureal(u_main, block_type, w_block, h_block, s_ball_mid_temp)
                        #     # Restore the u_move
                        #     main_state = [u_move[0], u_move[1], w_block, h_block, u_move[2]]
                        #     bool_overlap, _ = CheckOverlap(main_state, u_support)
                        #     if bool_overlap == True:
                        #         pass
                        #     else:
                        #         break   
                        # print("check time end{}".format(time.time()))
                        # print(umain_length_fobj_tuple)

                    # 8) Simulate the main block and observe the traj. with roughly estimated supporting blocks
                    # state_input = ([[id1, x1, y1, theta1],[id2, x2, y2, theta2],...]) list
                    # previous input is removed but, we have to leave the previous local region's input
                    
                    # input the main block
                    state_input_localregion = []
                    state_input_localregion.append([input_id, u_move[0], u_move[1], u_move[2]])
                    # input the supporting block
                    if support_id == None:
                        pass
                    else: 
                        state_input_localregion.append([support_id, x_support, y_support, theta_support])
                    
                    # draw the figure with blocks
                    alpha = 1.15-(count_1st_loop + count_2nd_loop + count_3rd_loop + count_4th_loop)*0.05
                    if not obj_main_pre:
                        print("no obj before")
                        pass
                    else:
                        display.eraseobject()


                    print("time check {}".format(time.time()))
                    if block_type == "metalrectangle" or block_type == "metalrtriangle":
                        metalblock = metalBlock(u_move[0],u_move[1],w_block,h_block,u_move[2])
                        obj_main_pre = display.drawobject(metalblock,alpha)
                    elif block_type == "woodrectangle" or block_type == "woodrtriangle":
                        woodblock = woodBlock(u_move[0],u_move[1],w_block,h_block,u_move[2])     
                        obj_main_pre = display.drawobject(woodblock,alpha)
                    if support_type == "metalrectangle" or support_type == "metalrtriangle":
                        metalblock = metalBlock(x_support,y_support,w_support,h_support,theta_support)
                        obj_support_pre = display.drawobject(metalblock, alpha)
                    elif support_type == "woodrectangle" or support_type == "woodrtriangle":
                        woodblock = woodBlock(x_support,y_support,w_support,h_support,theta_support)
                        obj_support_pre = display.drawobject(woodblock, alpha)
                    elif support_type == None:
                        pass
                    print("time check 2: {}".format(time.time()))
                    plt.show(block=False)
                    plt.pause(5)
                    state_input.extend(state_input_localregion)
                    print("time check 3: {}".format(time.time()))
                    print("state input {}".format(state_input))
                    run_simulation(level_select, movable_ID, ID_dict, state_input)
                    print("time check 4: {}".format(time.time()))

                    # clear the current input 
                    for i in range(len(state_input_localregion)):
                        del state_input[-1]

                    # Observe again about the main, ball, support blocks trajectory and 
                    # check when the contact between blocks is active (contact_idx)
                    traj_list, trace_time, collision, n_obj, bool_success, ID_list, _, _ = logging_trajectory("bb", level_select)   
                    # traj : [vx,vy,w,x,y,rot]
                    # check the order of the ID in the ID_list 
                    mainblock_traj = traj_list[pick_main_idx+1] # in log file, we include ball, so we need to add 1
                    ball_traj = traj_list[ball_idx]    
                    if support_idx == None:
                        support_block_traj = []
                    else:
                        support_block_traj = traj_list[support_idx+1]
                    contact_idx_blocks = 0
                    contact_idx_ball2main = 0
                    flag_collision_blocks = False
                    flag_collision_ball = False
                    for collision_element in collision:
                        # [time, collision type, objA, objB]
                        if (collision_element[2] == input_id and collision_element[3] != id_ball and flag_collision_blocks == False) or (collision_element[3] == input_id and collision_element[2] != id_ball and flag_collision_blocks == False):
                            if collision_element[1] == "collisionStart":
                                contact_idx_blocks = 0
                                flag_collision_blocks = True
                                while abs(collision_element[0] - trace_time[contact_idx_blocks]) > 10:
                                    contact_idx_blocks = contact_idx_blocks+1            
                        elif (collision_element[2] == input_id and collision_element[3] == id_ball and flag_collision_ball == False) or (collision_element[3] == input_id and collision_element[2] == id_ball and flag_collision_ball == False):
                            if collision_element[1] == "collisionStart":
                                contact_idx_ball2main = 0
                                flag_collision_ball = True
                                while abs(collision_element[0] - trace_time[contact_idx_ball2main]) > 10:
                                    contact_idx_ball2main = contact_idx_ball2main+1

                    # 9) Check the error between guide_path_local, direction_end and mainblock_traj at x_local  
                    # ref : [x,y,dir] / state_ball : [x,y,vx,vy]
                    ref = [guide_path_local_update[-1][0],guide_path_local_update[-1][1], direction_end]
                    traj_sorted, idx_sorted = SortedTrajInLocal(ball_traj, local_region)
                    
                    error_min = 50000
                    index_min = 0
                    for i in range(len(traj_sorted)):
                        state_ball_current = [traj_sorted[i][3], traj_sorted[i][4], traj_sorted[i][0], traj_sorted[i][1]]
                        error, error_vector = GetError(ref, state_ball_current)  
                        if error <= error_min:
                            error_min = error
                            index_min = i

                    if error_min == 50000: # need to change supporting blocks
                        # we need to add the way to change support block
                        break 
                    else:
                        print("error error_min {}, {}".format(error, error_min))
                        print("error, index_min, len(traj_sorted): {}, {}. {}".format(error, index_min, len(traj_sorted)))        

                    # 10) Check deviation between the actual traj. of main block and desired traj. of main block
                    # desired_traj_main [vx,vy,w,x,y,rot]
                    desired_traj_main = np.array([vel_desired[0], vel_desired[1], vel_desired[2], u_actual[0], u_actual[1], u_actual[2]])
                    error_main_array = desired_traj_main - mainblock_traj[contact_idx_ball2main]
                    err_main = np.sum(np.square(error_main_array))

                    print(mainblock_traj)
                    print("error of ball and main are {0} and {1}".format(error_min, err_main))
                    print("ball state at min error is {}".format(traj_sorted[index_min]))
                    print("main block state in collision is {}".format(mainblock_traj[contact_idx_ball2main]))
                    #display.drawtraj([traj_sorted[index_min]])
                    # draw the actual main block state at the contact with the ball
                    display.drawtraj(traj_sorted)
                    woodblock = woodBlock(mainblock_traj[contact_idx_ball2main][3],mainblock_traj[contact_idx_ball2main][4],w_block,h_block,mainblock_traj[contact_idx_ball2main][5])
                    display.drawobject(woodblock, 0.2)

                    plt.show(block=False)
                    plt.pause(3)

                    # 11) Compute the required force and point (if wood)
                    if block_type[0:5] == "metal":
                        pass
                    else:
                        print("contact_idx_blocks {}".format(contact_idx_blocks))
                        p2 = ComputeSupportForces(block_type, mainblock_desired_state, mainblock_traj, contact_idx_blocks)
                        #desired_state(x,y,w,h,rot,vx,vy,w_rot)
                    print(p2)
                    display.drawpoint(p2)
                    plt.show(block=False)
                    plt.pause(3)
                



    # 11) Error bound is satisfied --> success / violated --> update














