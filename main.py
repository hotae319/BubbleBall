import numpy as np
from matplotlib import pyplot as plt
from math import pi, cos, sin

from functions.path_planning import path_planning, re_planning
import functions.get_keydirection as keydirection
from functions.obj_class import Ball, metalBlock, woodBlock, powerUps
from functions.parsing_movableobjects_levels import parsing_objects, run_simulation
from functions.prediction import CheckCollisionWithFixed, ImpactMapBall2Fixed, ImpactMapBall2Wood
from functions.const import g, dt
'''
0. import the enviornment setting
'''
level = 21
id_grd, s_grd, s_total, id_total, n_total, movable_ID, ID_dict, ID_state_matching = parsing_objects(level)
# s_total = [s_ball, s_flag, s_metal, s_wood, s_woodtri, s_speedupu, s_speedupd, s_speedupl, s_speedupr, s_slowdown, s_gravity, s_gravitydown, s_spring, s_teleport]
# s_ball = [x,y,w,h,rot]
'''
1. path planning
'''
n_sample = 500
map_size = [0,500,0,500]
k1 = 6
k2 = 10
n_exclude = 1
prm, shortest_path, sampling_list = path_planning(level, n_sample, k1 , map_size)    
#re_planning(prm, level, shortest_path, k2, n_exclude)


'''
2. model prediction
'''
# Initialize
ball = Ball(s_total[0][0],s_total[0][1],0,0)
flag = s_total[1]
# Iterative update
for i in range(150):
    # Check collision
    collision = CheckCollisionWithFixed(ball, s_grd)
    print("{0} and pos: {1}".format(collision, [ball.x,ball.y]))
    # Update
    if not collision:       
        ball.update([0,ball.m*g])   
        ball.trajectory()   
    else:
        # suppose we get the first element
        idx_collision = collision[0]
        x_grd = s_grd[idx_collision][0]
        y_grd = s_grd[idx_collision][1]
        w_grd = s_grd[idx_collision][2]
        h_grd = s_grd[idx_collision][3]
        rot_grd = s_grd[idx_collision][4]/180*pi
        f = ImpactMapBall2Fixed(ball, rot_grd)        
        ball.update([f[0],f[1]+ball.m*g])
        ball.trajectory()   
        
traj = ball.trajectory()
xtraj = []
ytraj = []
for i in range(len(traj)):
    xtraj.append(traj[i][0])
    ytraj.append(traj[i][1])
plt.scatter(xtraj,ytraj, s = 2, c = 'green')
plt.show()
