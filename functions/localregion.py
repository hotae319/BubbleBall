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
import os, sys
if __name__ == "__main__":
    from parsing_movableobjects_levels import parsing_objects, run_simulation, logging_trajectory
    from physics.obj_class import Ball, metalBlock, woodBlock, powerUps
    from physics.simple_models import Ball2LineValue, Ball2CircleValue, BallinAirValue, BallinEnvValue
    from physics.common.const import g, dt
else:
    from . parsing_movableobjects_levels import parsing_objects, run_simulation, logging_trajectory
    from . physics.obj_class import Ball, metalBlock, woodBlock, powerUps
    from . physics.simple_models import Ball2LineValue, Ball2CircleValue, BallinAirValue, BallinEnvValue
    from . physics.common.const import g, dt


def SelectLocalRegion(guide_path, x_predicted_traj, y_predicted_traj, error_threshold = 10):
    # predicted traj : xtraj = [1,2,...], ytraj = [2,3,...]
    # guide_path(shortest_path): [[1,2][2,3]...]
    num_pred_traj = len(x_predicted_traj)
    num_guide_path = len(guide_path)
    dist_list = []
    error_list = []
    sum_dist = 0    
    x_predicted_traj_new = []
    y_predicted_traj_new = []
    for i in range(num_guide_path-1):
        xdist = guide_path[i][0]-guide_path[i+1][0]
        ydist = guide_path[i][1]-guide_path[i+1][1]
        dist = sqrt(xdist**2+ydist**2)
        sum_dist += dist
        dist_list.append(sum_dist)      
    # pick the pts of predicted traj to compute the tracking error
    # The entire predicted traj. is split into several parts which are proportional to interval of Tg
    for i in range(num_guide_path-1):
        id_pick = int(num_pred_traj*dist_list[i]/sum_dist)-1
        x_predicted_traj_new.append(x_predicted_traj[id_pick])
        y_predicted_traj_new.append(y_predicted_traj[id_pick])
        error_list.append(sqrt((guide_path[i][0]-x_predicted_traj_new[i])**2+(guide_path[i][1]-y_predicted_traj_new[i])**2))
    # new index for new list (the number is same with the guide path)
    idx_local_start = 0
    idx_local_end = 0
    while error_list[idx_local_start] < error_threshold/4:
        idx_local_start += 1
    while error_list[idx_local_end] < error_threshold:
        idx_local_end += 1
    # Decide the local region
    # id_pick : predicted traj's idx (outside the function, we need to choose s_ball_init)
    idx_pick_start = int(num_pred_traj*dist_list[idx_local_start]/sum_dist)
    idx_pick_end = int(num_pred_traj*dist_list[idx_local_end]/sum_dist)

    guide_path_local = guide_path[idx_local_start:idx_local_end+1] 
    # direction of the end element of local guide path, angle of atan(y/x)
    if guide_path[idx_local_end+1][0]-guide_path[idx_local_end][0] == 0:
        direction_end = pi/2*np.sign(guide_path[idx_local_end+1][1]-guide_path[idx_local_end][1])
    else:
        direction_end = atan((guide_path[idx_local_end+1][1]-guide_path[idx_local_end][1])/(guide_path[idx_local_end+1][0]-guide_path[idx_local_end][0]))
    # It can be changed to the total prediction traj
    x_pred_max = max(x_predicted_traj_new)
    y_pred_max = max(y_predicted_traj_new)
    x_guide_max = guide_path_local[idx_local_end-idx_local_start][0]
    y_guide_max = guide_path_local[idx_local_end-idx_local_start][1]
    x_local = x_guide_max
    #x_local = max(x_pred_max,x_guide_max)
    y_local = max(y_pred_max,y_guide_max)
    return guide_path_local, direction_end, x_local, y_local, idx_pick_start, idx_pick_end

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
    # x is the optimization variable

    # Common things irrelevant to the type of block
    # y[0] = ball.x, y[1] = ball.y, y[2] = ball.vx, y[3] = ball.vy, y[4] = ball.r
    # y[5] = guide_path_local[end][0] : x of guide path, y[6] : y of guide path
    # y[7] : direction of v, vx/vy of guide path
    # y[8] : block type, y[9] : block state [x,y,w,h,rot]

    # tracking error (ref = [x,y,dir])
    ref = [y[5],y[6],y[7]]  

    # All rollouts consist of "gravity or environment -> input -> gravity or environment"
    if y[8] == "metalrectangle" or "woodrectangle":
        # 1) retangle
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w
        #  l should be lower than y[9][3] = w 
        state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4],x[0],x[1],x[2],x[3],x[4])
    elif y[8] == "woodcircle" or "metalcircle":
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
    if y[8] == "metalrectangle" or "woodrectangle":
        # 1) retangle
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w
        #  l should be lower than y[9][3] = w 
        state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4],x[0],x[1],x[2],x[3],x[4])
    elif y[8] == "woodcircle" or "metalcircle":
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
    if block_type == "metalrectangle" or "woodrectangle":
        # u_input = [ l, theta, vx, vy, w]
        l = u_input[0]
        width = w_block
        height = h_block
        theta = u_input[1]/180*pi
        u_actual = [xball+rball-rball*sin(theta)+l*cos(theta)-width/2-width/2*cos(theta)-height/2*sin(theta), yball+rball+rball*cos(theta)+l*sin(theta)-height/2-width/2*sin(theta)+height/2*cos(theta), u_input[1]]
        vel_desired = [u_input[2],u_input[3],u_input[4]]
    elif block_type == "woodcircle" or "metalcircle":
        # u_input = [x, y, vx, vy, w]
        u_actual = [u_input[0],u_input[1],0]
        vel_desired = [u_input[2],u_input[3],u_input[4]]
    return u_actual, vel_desired

def FindOptimalInput(guide_path_local, direction_end, block_type, block_state, s_ball_ini):
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
    # Given parameters
    # y = (ball(5),ref(3),block type, block state)
    y = (s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],s_ball_ini[4], ref[0],ref[1],ref[2], block_type, block_state)        
    if block_type == "metalrectangle" or "woodrectangle":
        # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w
        # block_state = [x,y,w,h,rot]
        x0 = [130,0,0,0,0]        
        # bound : 0 < l < width, -90<theta<90
        search_bound = Bounds([0,-90,-np.inf,-np.inf,-np.inf],[block_state[2],90,np.inf,np.inf,np.inf])
        # nonlinear constr : l and theta have same sign
        f_nonlin = lambda x:x[0]*x[1]
        nonlin_constr = NonlinearConstraint(f_nonlin,0,np.inf)
        # solve optimization
        res = minimize(fobj ,x0, args = y, bounds = search_bound, constraints = nonlin_constr)
        print(res)                
        u_input = res.x
    elif block_type == "woodcircle" or "metalcircle":
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

def ComputeSupportForces(mainblock_type, mainblock_desired_state, main_traj,  contact_idx):
    # Compute the 2 forces or 1 force with a fixed force, given the desired state of a main block
    # Input : type("rectangle"), desired_state(x,y,w,h,rot,vx,vy,w_rot)
    #         current trajectory : main_traj([[vx,vy,w,x,y,rot],[],...,[]])
    #         contact_point : [x,y], contact_idx : index of traj. when the contact starts
    # Output : two points we will support (p1[x1,y1], p2[x2,y2]) / one point or range corresponding to given contact pt
    # the supporting forces which can compensate the movement
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
        fx = m*(main_traj[i+1][0]-main_traj[i][0])/dt         
        fy = m*(main_traj[i+1][1]-main_traj[i][1])/dt
        torque = I*(main_traj[i+1][2]-main_traj[i][2])/dt
        fx_list.append(fx)
        fy_list.append(fy)
        torque_list.append(torque)
    # Actual forces/torques when contact starts
    fx1 = fx_list[contact_idx]
    fy1 = fy_list[contact_idx]
    torque1 = torque_list[contact_idx]
    # Compute the desired force/torque (F = m(vref-0)/dt)
    fx_desired = m*mainblock_desired_state[5]/dt
    fy_desired = m*mainblock_desired_state[6]/dt
    torque_desired = I*mainblock_desired_state[7]/dt
    fx2 = fx_desired - fx1 
    fy2 = fy_desired - fy1 - m*g
    torque2 = torque_desired - torque1
    # roughly compute (xcm+lx, ycm)
    lx = torque2/fy2
    x2 = mainblock_desired_state[0] + mainblock_desired_state[2]/2 + lx
    y2 = mainblock_desired_state[1] + mainblock_desired_state[3]/2 
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
    def __init__(self, xregion, yregion, xsize, ysize, s_grd_list, guide_path_local, s_ball_ini):
        self.xregion = xregion
        self.yregion = yregion
        self.xsize = xsize
        self.ysize = ysize
        self.s_grd_list = s_grd_list
        self.guide_path_local = guide_path_local
        self.s_ball_ini = s_ball_ini
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
        xpath = [self.guide_path_local[i][0] for i in range(len(self.guide_path_local))]
        ypath = [self.guide_path_local[i][1] for i in range(len(self.guide_path_local))]
        # Draw the shortest path [[1,2],[2,3],...]
        self.ax1.plot(xpath,ypath, c = 'b', marker = 'o')

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
    def drawobject(self, obj):
        if obj.__class__.__name__ == "woodBlock":
            ts = self.ax1.transData
            tr = trans.Affine2D().rotate_deg_around(obj.x+obj.w/2,obj.y+obj.h/2, obj.rot)
            t = tr + ts # tr + ts (order is important)
            rect = patches.Rectangle((obj.x,obj.y),obj.w,obj.h, edgecolor='y', facecolor="y", transform = t)
            self.ax1.add_patch(rect) 
            self.ax1.plot()
        elif obj.__class__.__name__=="metalBlock":
            ts = self.ax1.transData
            tr = trans.Affine2D().rotate_deg_around(obj.x+obj.w/2,obj.y+obj.h/2, obj.rot)
            t = tr + ts # tr + ts (order is important)
            rect = patches.Rectangle((obj.x,obj.y),obj.w,obj.h, edgecolor='gray', facecolor="gray", transform = t)
            self.ax1.add_patch(rect) 
            self.ax1.plot()
        return self.ax1

def ff(x,*y):    
    a = x[0]
    b = x[1]**2+y[0]*x[1]+y[1]
    r = np.array([a,b])
    return r


if __name__=="__main__":
    # s_ball_ini = [60,80,0,5,15] # s_ball_ini (x,y,vx,vy,r)
    # ref = [100,100,0.3]
    # block_type = "rectangle"
    # block_state = [0,0,50,150,0]
    # model_error = np.array([10,10,0.1])
    # # FindOptimalInput(guide_path_local, direction_end, block_type, block_state, s_ball_ini):
    # u = FindOptimalInput([[60,80],[100,100]], 0.3, "rectangle", [0,0,50,150,0],[60,80,0,5,15])
    # #u = FindOptimalInput([[60,80],[100,100]], 0.3, "circle", [0,0,50,50,0],[60,80,0,5,15])
    # print("u is {}".format(u))
    # y = (s_ball_ini[0],s_ball_ini[1],s_ball_ini[2],s_ball_ini[3],s_ball_ini[4], ref[0],ref[1],ref[2], block_type, block_state)
    # f_actual = fmodel(u,*y)+model_error
    # error = ref - f_actual
    # e = np.matmul(error,error)
    # print("error is {}".format(e))
    # print(f_actual)
    # #a = np.array([1,2])
    # #print(ff(a,*(2,3)))
    # #f = nd.Jacobian(ff)
    # #print(f(a,*(4,3)))
    # unew = UpdateModelAfterFailure([[60,80],[100,100]], 0.3, "rectangle", [0,0,50,150,0],[60,80,0,5,15], u, f_actual)
    # print("new u is {}".format(unew))
    # f_new = fmodel(unew,*y)+model_error
    # print(f_new)
    # unew1 = UpdateModelAfterFailure([[60,80],[100,100]], 0.3, "rectangle", [0,0,50,150,0],[60,80,0,5,15], unew, f_new)
    # print("new u is {}".format(unew1))
    # f_new1 = fmodel(unew1,*y)+model_error
    # print(f_new1)

    # # Draw local region 
    # xregion = 100
    # yregion = 100
    # xsize = 200
    # ysize = 150
    # s_grd_list = [[60,150,80,10,0],[150,170,90,15,10]]
    # guide_path_local = [[100,100],[150,110],[200,150],[300,160]]
    # s_ball_ini = [100,100,10,0,15]

    # # woodblock state
    # w = 100
    # h = 20

    # x = 150
    # y = 130
    # rot = 5
    # vx = 0
    # vy = 0
    # w_rot = 0
    # block_type = "rectangle"

    # block1 = woodBlock(x, y, w, h, rot, vx, vy, w_rot, block_type)
    # display = Drawing(xregion, yregion, xsize, ysize, s_grd_list, guide_path_local, s_ball_ini)
    # display.drawlocalregion()
    # display.drawobject(block1)
    # plt.show()


    # Test for a local region
    level_select = 2
    # 0) Prerequisite : guide path
    guide_path = [[10,10],[10,20],[30,30]]
    state_input = [] # decided ahead in previous local regions
    state_input.append(["AO6G", 0,0,0])
    # 1) Predict the ball's traj, load a traj. from a log file
    id_grd, s_grd_list, s_total, id_total, n_total, movable_ID, ID_dict, ID_state_matching = parsing_objects(level_select)
    #run_simulation(level_select, movable_ID, ID_dict, state_input)
    traj_list, trace_time, collision, n_obj, bool_success, ID_list, _, _ = logging_trajectory("bb", 2)       
    id_ball = id_total[0]
    s_ball = s_total[0] # [x,y,w,h,rot], we use only w and h here
    # check the order of the ID in the ID_list
    ball_idx = 0
    while id_ball != ID_list[ball_idx] and ball_idx < len(ID_list):
        ball_idx = ball_idx+1
    ball_traj = traj_list[ball_idx] # [vx,vy,wrot,x,y,theta] 6 by N
    # 2) Choose a local region and cut-off the guide path locally
    x_predicted_traj = [ball_traj[j][3] for j in range(len(ball_traj))]
    y_predicted_traj = [ball_traj[j][4] for j in range(len(ball_traj))]
    guide_path_local, direction_end, x_local, y_local, idx_pick_start, idx_pick_end = SelectLocalRegion(guide_path, x_predicted_traj, y_predicted_traj)
    xregion = min(guide_path_local[0][0],x_predicted_traj[idx_pick_start])
    yregion = min(guide_path_local[0][1],x_predicted_traj[idx_pick_end])
    xsize = x_local-xregion
    ysize = y_local-yregion
    # s_ball_ini = [x,y,vx,vy,r] : so we have to change the order because log file has vx,vy first
    s_ball_ini = [ball_traj[idx_pick_start][3],ball_traj[idx_pick_start][4],ball_traj[idx_pick_start][0],ball_traj[idx_pick_start][1],15]
    
    display = Drawing(xregion, yregion, xsize, ysize, s_grd_list, guide_path, s_ball_ini)
    display.drawlocalregion()
    plt.show(block=False)
    plt.pause(5)    

    # While the local region fails, do a loop
    pick_main_idx = 0
    # 3) Pick a main block / we can add some priorities    
    input_id = movable_ID[pick_main_idx]
    block_type = ID_dict[input_id] # 'metalrectangle'
    block_state = ID_state_matching[input_id] # [x,y,w,h,rot]
    # 4) Solve the optimization problem to obtain the required state of the main block
    u_optimal= FindOptimalInput(guide_path_local, direction_end, block_type, block_state, s_ball_ini)
    w_block = block_state[2]
    h_block = block_state[3]
    u_actual, vel_desired = ConvertUopt2Ureal(u_optimal, block_type, w_block, h_block, s_ball_ini)
    desired_state = [u_actual[0],u_actual[1],w_block,h_block,u_actual[2],vel_desired[0],vel_desired[1],vel_desired[2]]
    # 5) Simulate the main block and observe the traj. without supporting blocks
    # state_input = ([[id1, x1, y1, theta1],[id2, x2, y2, theta2],...]) list
    del state_input[-1]
    state_input.append([input_id, u_actual[0], u_actual[1], u_actual[2]])
    metalrect = metalBlock(u_actual[0],u_actual[1],w_block,h_block,u_actual[2])
    display.drawobject(metalrect)
    plt.show(block=False)
    plt.pause(15)
    #run_simulation(level_select, movable_ID, ID_dict, state_input)
    # observe again   
    traj_list, trace_time, collision, n_obj, bool_success, ID_list, _, _ = logging_trajectory("bb", 2)   
    mainblock_traj = traj_list[pick_main_idx]
    for collision_element in collision:
        # [time, collision type, objA, objB]
        if collision_element[2] == input_id or collision_element[3] == input_id:
            if collision_element[1] == "collisionStart":
                contact_idx = 0
                while abs(collision_element[0] - trace_time[contact_idx]) > 10:
                    contact_idx = contact_idx+1
    # 6) Compute the required force and point (if wood)
    if block_type[0:5] == "metal":
        pass
    else:
        p2 = ComputeSupportForces(block_type, mainblock_desired_state, mainblock_traj, contact_idx)
        #desired_state(x,y,w,h,rot,vx,vy,w_rot)
    # 7) Assign the supporting blocks
    # 8) Observe the ball's traj.
    # 9) Check the error between guide_path_local and mainblock_traj at x_local    
    # 10) Error bound is satisfied --> success / violated --> update
    # f_actual = [x,y,dir]

    #unew = UpdateModelAfterFailure(guide_path_local, direction_end, block_type, block_state, s_ball_ini, u_optimal, f_actual)











