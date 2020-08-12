# Model prediction and Compute distance cost
from math import cos, sin, sqrt, atan, pi
if __name__ == "__main__":
    from const import g, dt
    from planning_algo.utils import CheckInside, CheckIntersect, CheckIntersectPolygon, RotatePts
else:
    from . const import g, dt
    from . planning_algo.utils import CheckInside, CheckIntersect, CheckIntersectPolygon, RotatePts, GetDistance

def CheckCollisionWithFixed(ball, block_list, n_timestep):
    # state_list = [block1, block2, ...] / block_k = [x,y,w,h,rot]
    # ball's path line
    # suppose the rectangle (circle : no rot, triangle : exclude one point)
    
    ball_pre = [ball.x, ball.y]
    ball_post = ball.updateValue([0,ball.m*g], n_timestep)[0:2]
    bool_intersect = []
    angle = 0
    for i in range(len(block_list)):
        if block_list[i][4] == 0:
            x = block_list[i][0]
            y = block_list[i][1]
            w = block_list[i][2]
            h = block_list[i][3]
            pts = [[x,y],[x+w,y],[x+w,y+h],[x,y+h]]
        else:
            pts = RotatePts(block_list[i])
        check, line, pt_int = CheckIntersectPolygon(pts, ball_pre, ball_post)
        if check == True:   
            bool_intersect.append(i)
            # if single intersection exists
            if len(line) == 1:                    
                if line[0][0][0]-line[0][1][0] == 0:
                    angle = pi/2
                elif line[0][0][1]-line[0][1][1] == 0:
                    angle = 0
                else:
                    angle = atan((line[0][0][1]-line[0][1][1])/(line[0][0][0]-line[0][1][0]))
            else:
                # if multiple intersections exist
                dist = GetDistance(pt_int[0], ball_pre)
                idx_dist = 0
                print("ballpre, ballpost: {2}, {3} inter_pt, line: {0}, {1}".format(pt_int[0], line[idx_dist], ball_pre, ball_post))
                for j in range(len(pt_int)-1):
                    if GetDistance(pt_int[j+1], ball_pre) < dist:
                        dist = GetDistance(pt_int[j+1], ball_pre)
                        idx_dist = j+1     
                    print("inter_pt, line: {0}, {1}".format(pt_int[j+1], line[idx_dist]))
                if line[idx_dist][0][0]-line[idx_dist][1][0] == 0:
                    angle = pi/2
                elif line[idx_dist][0][1]-line[idx_dist][1][1] == 0:
                    angle = 0
                else:
                    angle = atan((line[idx_dist][0][1]-line[idx_dist][1][1])/(line[idx_dist][0][0]-line[idx_dist][1][0]))
            break
    return bool_intersect, angle  
def ImpactMapBall2Fixed(ball, rot):
    # line = [p1, p2] / ball is a class / line shows the direction of collision
    if rot == 0:
        v1 = ball.vy
        f_normal = -ball.m*(ball.e+1)/dt*v1-ball.m*g
        f = [0, f_normal]
    else:
        v1 = ball.vy*cos(rot)-ball.vx*sin(rot)
        f_normal = -ball.m*(ball.e+1)/dt*v1-ball.m*g*cos(rot)
        f = [-f_normal*sin(rot), f_normal*cos(rot)]
    return f
def ImpactMapBall2Wood(ball, wood):
    l_collision = sqrt((ball.x-(wood.x+wood.w))**2+(ball.y-(wood.y+wood.h))**2)
    v1 = ball.vy*cos(rot)-ball.vx*sin(rot)
    v2 = wood.vy*cos(rot)-wood.vx*sin(rot)+wood.w*l_collision
    f_normal = -ball.m*(ball.e+1)/dt*(v1+v2)-ball.m*g*cos(rot)
    f = [-f_normal*sin(rot), f_normal*cos(rot)]
    return f
def ImpactMapWood2Fixed():
    a = 1
def ImpactMapWood2Wood():
    b = 1
def ComputeDistance(shortest_path, predicted_path):
    # We suppose both paths have the same nubmer of the waypoints
    # Use the simplest cost to compute the distnace between two paths (It can be changed later)
    dist = 0
    for i in range(len(shortest_path)):
        dist += abs(shortest_path[i]-predicted_path[i])
    return dist
