'''
    @ Author : Hotae Lee
    @ Date : 08/11/2020
    @ Function : simple models for dynamic equations wiht a rough assumption 
    @ Parameters : 
    @ Variables:  
    @ Retruns :  
    @ Description : 
    @ TODO : 
'''
import numpy as np
from scipy.optimize import minimize, Bounds, LinearConstraint, NonlinearConstraint
from math import cos, sin, sqrt, pi

if __name__ == "__main__":
    from common.const import g, dt    
    #from planning_algo.utils import RotatePts    
else:
    from . common.const import g, dt
    #from . planning_algo.utils import RotatePts

# A) Ball
# 1) ball to line (state: ball, input: l, theta, vx, vy, w)
def Ball2Line(ball, l, theta, vx, vy, w, v_thres = 0.01, w_thres = 0.01):
    vt = ball.vx*cos(theta)+ball.vy*sin(theta)
    v = vy*cos(theta)-vx*sin(theta)+l*w # v means the velocity of the contact point of the block 
    # rolling on the block
    if abs(v) <= v_thres and abs(w) <= w_tshres:
        ball.x = ball.x+l*cos(theta)
        ball.y = ball.y+l*sin(theta)
        vend = sqrt(vt**2+2*g*sin(theta))
        ball.vx = vend*cos(theta)
        ball.vy = vend*sin(theta)
        status = "rolling"
    # hitting the ball
    else:
        ball.vx = v*sin(theta)+vt*cos(theta)
        ball.vy = v*cos(theta)-vt*sin(theta)
        status = "hitting"
    return ball, status

def Ball2LineValue(xball, yball, vxball, vyball, rball, l, theta, vx, vy, w, v_thres = 1, w_thres = 1):
    # degree to radians 
    theta = theta/180*pi
    vt = vxball*cos(theta)+vyball*sin(theta)
    v = vy*cos(theta)-vx*sin(theta)+l*w # v means the velocity of the contact point of the block 
    # rolling on the block
    if abs(v) <= v_thres and abs(w) <= w_thres:
        if l*theta >= 0:            
            if abs(vt) >= 0.5:
                vend = sqrt(vt**2+2*g*l*sin(theta))*np.sign(l)
                xball = xball+l*cos(theta)
                yball = yball+l*sin(theta)
                vxball = vend*cos(theta)
                vyball = vend*sin(theta)      
                status = "rolling"          
            else:
                vxball = 0
                vyball = 0
                status = "stop"
        else:
            # l>0 , theta <0 or l<0, theta >0
            if vt*l < 0:
                vxball = vt*cos(theta)
                vyball = vt*sin(theta)
                status = "rolling"
            if vt**2+2*g*l*sin(theta) >0:
                vend = sqrt(vt**2+2*g*l*sin(theta))*np.sign(l)
                xball = xball+l*cos(theta)
                yball = yball+l*sin(theta)
                vxball = vend*cos(theta)
                vyball = vend*sin(theta)
                status = "rolling"
            else: # opposite slope
                vend = -vt # opposite
                status = "rolling"

    # hitting the ball
    else:
        vxball = v*sin(theta)+vt*cos(theta)
        vyball = v*cos(theta)-vt*sin(theta)
        status = "hitting"
    ball = [xball,yball,vxball,vyball]
    return ball, status

def Ball2MovingLineValue(xball, yball, vxball, vyball, rball, l, theta, vx, vy, w, e, v_thres = 1, w_thres = 1):
    # degree to radians 
    theta = theta/180*pi
    w = w/180*pi
    vt = vxball*cos(theta)+vyball*sin(theta)
    # v means the velocity of the contact point of the block 
    # velocity approaching each other
    v = -vy*cos(theta)+vx*sin(theta)+abs(l)*w 
    # after collision
    vxball = v*sin(theta)+vt*cos(theta)
    vyball = -v*cos(theta)+vt*sin(theta)    
    ball = [xball,yball,vxball,vyball]
    return ball

# 2) ball to circle (state: ball, input: x,y,v,w)
def Ball2Circle(ball, r, x, y, vx, vy, w, v_thres = 1):
    vt = ball.vx*(y-ball.y)/(ball.radius+r) - ball.vy*(x-ball.x)/(ball.radius+r)
    v = vy*(x-ball.x)/(ball.radius+r)-vx*(y-ball.y)/(ball.radius+r)
    sin_theta = (y-ball.y)/(ball.radius+r)
    cos_theta = (x-ball.x)/(ball.radius+r)
    if abs(v) <= v_thres:
        ball.vx = vt*sin_theta # vt*sin(theta)
        ball.vy = vt*cos_theta
    else:       
        ball.vx = vt*sin_theta-v*cos_theta # vt*sin(theta)
        ball.vy = vt*cos_theta-v*sin_theta
    return ball

def Ball2CircleValue(xball, yball, vxball, vyball, rball, r, x, y, vx, vy, w, e = 0, v_thres = 1):
    
    # e : coefficient of restitution
    vt_ball = vxball*(y+r-yball-rball)/(rball+r) - vyball*(x+r-xball-rball)/(rball+r)
    vn_ball = vxball*(x+r-xball-rball)/(rball+r) + vyball*(y+r-yball-rball)/(rball+r)
    # v : vel of Circle
    vt = vx*(y+r-yball-rball)/(rball+r) - vy*(x+r-xball-rball)/(rball+r)
    vn = vy*(x+r-xball-rball)/(rball+r) + vx*(y+r-yball-rball)/(rball+r)

    cos_theta = (y+r-yball-rball)/(rball+r)
    sin_theta = (x+r-xball-rball)/(rball+r)
    if abs(vn) <= v_thres:
        vxball = vt_ball*cos_theta - e*vn_ball*sin_theta # vt*sin(theta)
        vyball = -vt_ball*sin_theta - e*vn_ball*cos_theta
    else:       
        vxball = vt_ball*cos_theta-(vn+e*vn_ball)*sin_theta # vt*sin(theta)
        vyball = -vt_ball*sin_theta-(vn+e*vn_ball)*cos_theta
    ball = [xball,yball,vxball,vyball]
    return ball

def Ball2CircleValueTilde(xball, yball, vxball, vyball, rball, r, x, y, vx, vy, w, e = 0, v_thres = 1):
    # # e : coefficient of restitution
    # vt_ball = vxball*(y-yball)/(rball+r) - vyball*(x-xball)/(rball+r)
    # vn_ball = vxball*(x-xball)/(rball+r) + vyball*(y-yball)/(rball+r)
    # # v : vel of Circle
    # vt = vx*(y-yball)/(rball+r) - vy*(x-xball)/(rball+r)
    # vn = vy*(x-xball)/(rball+r) + vx*(y-yball)/(rball+r)

    # cos_theta = (y-yball)/(rball+r)
    # sin_theta = (x-xball)/(rball+r)
    # e : coefficient of restitution
    vt_ball = vxball*(y+r-yball-rball)/(rball+r) - vyball*(x+r-xball-rball)/(rball+r)
    vn_ball = vxball*(x+r-xball-rball)/(rball+r) + vyball*(y+r-yball-rball)/(rball+r)
    # v : vel of Circle
    vt = vx*(y+r-yball-rball)/(rball+r) - vy*(x+r-xball-rball)/(rball+r)
    vn = vy*(x+r-xball-rball)/(rball+r) + vx*(y+r-yball-rball)/(rball+r)

    cos_theta = (y+r-yball-rball)/(rball+r)
    sin_theta = (x+r-xball-rball)/(rball+r)
    if abs(vn) <= v_thres:
        vxball = - vn_ball*sin_theta # vt*sin(theta)
        vyball = - vn_ball*cos_theta
    else:       
        vxball = -(vn_ball)*sin_theta # vt*sin(theta)
        vyball = -(vn_ball)*cos_theta
    ball = [xball,yball,vxball,vyball]
    return ball

# 3) ball to point
def Ball2Point(ball, x, y):
    sin_theta = (y-ball.y)/ball.radius
    cos_theta = (x-ball.x)/ball.radius
    vt = ball.vy*sin_theta-ball.vx*cos_theta
    ball.vx = vt*cos_theta
    ball.vy = -vt*sin_theta
    return ball

def Ball2PointValue(xball, yball, vxball, vyball, rball, x, y):
    sin_theta = (y-yball)/rball
    cos_theta = (x-xball)/rball
    vt = vyball*sin_theta-vxball*cos_theta
    vxball = vt*cos_theta
    vyball = -vt*sin_theta
    ball = [xball,yball,vxball,vyball]
    return ball

# 4) ball in the air
def BallinAir(ball, l):
    ball.x += l
    ball.y += ball.vy/ball.vx*l+g/2*l**2/ball.vx**2
    ball.vy += g*l/ball.vx
    return ball

def BallinAirValue(xball, yball, vxball, vyball, lx, ly = 0):
    # l denotes the distance along x axis, l>0
    if lx == 0 or abs(vxball) < 0.5 :
        
        if vyball**2+2*g*ly >=0:
            yball += ly
            vyball += sqrt(vyball**2+2*g*ly)
        else:
            vyball = 0
            yball += -vyball**2/2/g

    else:
        if lx*vxball >= 0:
            xball += lx
            yball += vyball/vxball*lx+g/2*lx**2/vxball**2
            vyball += g*lx/vxball
        else:
            xball += lx
            yball = 10000000000
            vyball = 1000000000
    ball = [xball,yball,vxball,vyball]
    return ball


# 5) ball in the small region (combine air and ground)
def BallinEnvValue(xball, yball, vxball, vyball, rball, x, y, w, h, theta, xregion, yregion, xsize, ysize):
    # Start from right or left
    if xball-xregion > xregion+xsize-xball:
        #From right
        x0 = xregion+xsize        
        x3 = xregion
    else:
        #From left
        x0 = xregion
        x3 = xregion+xsize
    # Find the boundary
    theta = theta/180*pi
    if x+w/2-w/2*cos(theta)+h/2*sin(theta) < x0:
        xleft = xregion
    else:
        xleft = x+w/2-w/2*cos(theta)+h/2*sin(theta)        
    if x+w/2+w*cos(theta) > x0+xsize:
        xright = xregion+xsize
    else:
        xright = x+w/2+w*cos(theta)
    # Decide x1, x2
    if x0 == xball:
        x1 = xleft
        x2 = xright
    else:
        x1 = xright
        x2 = xleft
    # Check collisions at x1 / roughly predict vel at x1 (assume the collision at x1)
    if x1 == x0:        
        y1 = y+h/2-(x+w/2-x0)*tan(theta)-h/2*cos(theta)
        state1 = BallinAirValue(x0, yabll, vxball, vyball, 0, y1-yball)
        state1 = [x1, y1, vxball, state1[3]]
        # get the state at x2 
        l = (x2-x1)/cos(theta)
        state2, _ = Ball2LineValue(state1[0], state1[1], state1[2], state1[3], 15, l, theta*180/pi, 0, 0, 0)
        if x2 == x3:
            state3 = state2
        else:
            state3 = BallinAirValue(state2[0], state2[1], state2[2], state2[3], x3-x2)
    else:
        y1 = y+h/2-w/2*abs(sin(theta))-h/2*cos(theta)
        state1 = BallinAirValue(x0, yball, vxball, vyball, x1-x0)
        if y1 > state1[1]:
            # collision happens
            state1 = BallinAirValue(x0, yball, vxball, vyball, x1-x0)
            state1 = [x1, y1, vxball, state1[3]]
            # get the state at x2 
            l = (x2-x1)/cos(theta)
            state2, _ = Ball2LineValue(state1[0], state1[1], state1[2], state1[3], 15, l, theta*180/pi, 0, 0, 0)
            if x2 == x3:
                state3 = state2
            else:
                state3 = BallinAirValue(state2[0], state2[1], state2[2], state2[3], x3-x2)
        else:
            # no collision happens
            state2 = state1
            state3 = BallinAirValue(x0, yball, vxball, vyball, x3-x0, ysize)
            print("no contact")

    return state3, state2, state1

    #6) Ball with speedupr
def Ball2PowerupValue(xball, yball, vxball, vyball, rball, x, y, power_type):
    r_powerup = 25
    m = 4/3*pi*15**3*0.2
    if power_type == "speedupr":
        if (x+r_powerup-xball-rball)**2+(y+r_powerup-yball-rball)**2 <= (r_powerup+rball)**2:
            vxball += 191
        else:
            vxball += 0
    elif power_type == "speedupl":
        if (x+r_powerup-xball-rball)**2+(y+r_powerup-yball-rball)**2 <= (r_powerup+rball)**2:
            vxball -= 191
        else:
            vxball -= 0
    ball = [xball,yball,vxball,vyball]
    return ball

# B) Blocks
# 1) 1 point to line (pivot point : px, py)
def Point2Line(px,py,block1,block2):
    xcm = block1.x+block1.w/2
    ycm = block1.y+block1.h/2
    theta = block2.rot
    d_left = block1.h # need to check
    d_right = block1.w
    if xcm>px: 
        # falling to the right
        block1.x = px + d_right*cos(theta)/2 - d_left*sin(theta)/2 - d_right/2
        block1.y = py - d_right*sin(theta)/2 - d_left*cos(theta)/2 - d_left/2
        block1.rot = block2.rot
    else:
        # falling to the left
        block1.x = px - d_left*cos(theta)/2 - d_right*sin(theta)/2 - d_right/2
        block1.y = py - d_right*cos(theta)/2 + d_left*sin(theta)/2 - d_left/2
        block1.rot = block2.rot + 90

    if abs(block2.vx) < 0.05 and abs(block2.vy) < 0.05 and abs(block2.w_rot) < 1:
        # The upper block is stopped while attching to the lower block.
        status = "attched"
    else:
        # angular momentum conservation
        m1 = block1.den*block1.w*block1.h
        m2 = block2.den*blcok2.w*block2.h
        I1 = m1/12*(block1.w**2+block1.h**2)
        I2 = m2/12*(block2.w**2+block2.h**2)
        vnew_x = (m1*block1.vx + m2*block2.vx)/(m1+m2)
        vnew_y = (m1*block1.vy + m2*block2.vy)/(m1+m2)
        l1 = 0
        l2 = 0
        wnew = (I1*block1.w_rot+I2*blcok2.w_rot)/(I1+I2+m1*l1**2+m2*l2**2)
        block1.vx = vnew_x
        block2.vx = vnew_x
        block1.vy = vnew_y
        block2.vy = vnew_y
        block1.w_rot = wnew
        block2.w_rot = wnew
        status = "moving"
    return block1, block2, status
# 2) 2pts to line (pivot point : px1, py1, px2, py2)
def Points2Line(px1,py1,px2,py2,block, block1, block2):
    # block1 which contacts the main block at p1
    # block2 which contacts the main block at p2
    xcm = block.x+block.w/2
    ycm = block.y+block.h/2
    if (px1-xcm)*(px2-xcm) > 0:
        # the point close to C.M. becomes the pivot point.
        if abs(px1-xcm) < abs(px2-xcm):
            # p1 can be a pivot
            block, block1 = Point2Line(px1, py1, block, block1)
        else:
            # p2 can be a pviot
            block, block2 = Point2Line(px1, py1, block, block2)            
    return block, block1, block2
# 3) line to circle
def Line2Circle(block1, block2, px, py, l):
    # block1 should be a line, upper block
    # l : the distance along the side(the direction of the block)
    xcm1 = block1.x+block1.w/2
    ycm1 = block1.y+block1.h/2    
    xcm2 = block2.x+block2.w/2
    ycm2 = block2.y+block2.h/2   
    d_left = block1.h # need to check
    d_right = block1.w
    vt = vx*cos(block1.rot)+vy*sin(block1.rot)
    if xcm1 > xcm2:
        # move to the right
        block1.x += l*cos(block1.rot)
        block1.y += l*sin(block1.rot)
        vend = sqrt(2*g*sin(block1.rot)+vt**2)
        block1.vx = vend*cos(block1.rot)
        block1.vy = vend*sin(block1.rot)
    else:
        # move to the left
        block1.x -= l*cos(block1.rot)
        block1.y -= l*sin(block1.rot)
        vend = sqrt(2*g*sin(block1.rot)+vt**2)
        block1.vx = -vend*cos(block1.rot)
        block1.vy = -vend*sin(block1.rot)
    return block1
# 4) Circle to line 
def Circle2Line(block1, block2, l, vx, vy, w):
    # block1 should be a circle
    theta = block2.rot
    vt = block1.vx*cos(theta)+block1.vy*sin(theta)
    v = vy*cos(theta)-vx*sin(theta)+l*w # v means the velocity of the contact point of the block 
    # rolling on the block
    block1.x = block1.x+l*cos(theta)
    block1.y = block1.y+l*sin(theta)
    vend = sqrt(vt**2+2*g/3*cos(theta)) # reduced acceleration
    block1.vx = vend*cos(theta)
    block1.vy = vend*sin(theta)
    block1.w_rot = vend/block1.radius
    return block1

def f(x,*y):
    # x[0] = l, x[1] = theta, x[2] = vx, x[3] = vy, x[4] = w
    # y[0] = ball.x, y[1] = ball.y, y[2] = ball.vx, y[3] = ball.vy, y[4] = ball.r
    state_ball, _ = Ball2LineValue(y[0],y[1],y[2],y[3],y[4],x[0],x[1],x[2],x[3],x[4])
    error = (state_ball[0]-100)**2+(state_ball[1]-100)**2+0*state_ball[2]**2+0*state_ball[3]**2 
    return error


if __name__ == "__main__":
    #ball, status = Ball2Line(ball, l, theta, vx, vy, w, v_thres = 0.01, w_thres = 0.01)
    x0 = [10,0,0,0,0]
    y = (50,50,0,15,15)
    search_bound = Bounds([0,-90,-np.inf,-np.inf,-np.inf],[np.inf,90,np.inf,np.inf,np.inf])
    f_nonlin = lambda x:x[0]*x[1]
    nonlin_constr = NonlinearConstraint(f_nonlin,0,np.inf)
    res = minimize(f,x0, args = y, bounds = search_bound, constraints = nonlin_constr)
    print(res.x, res.fun)
    import os, sys

    abspath = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
    print(abspath)

    xball = 0
    yball = 0
    vxball = 30
    vyball = 5
    rball = 15
    xregion = 0
    yregion = 0
    xsize = 50
    ysize = 50
    x = 10
    y = 30
    w = 35
    h = 5 
    theta = 10

    state3, state2, state1 = BallinEnvValue(xball, yball, vxball, vyball, rball, x, y, w, h, theta, xregion, yregion, xsize, ysize)
    print(state3, state2, state1)