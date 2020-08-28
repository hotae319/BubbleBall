# utils
import numpy as np
from math import sqrt, pi, atan, cos, sin
from matplotlib import pyplot as plt

def GetDistance(p1,p2):
    dist = sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)
    return dist

def LineFrom2pts(p1,p2):
    a = p1[1]-p2[1]
    b = p2[0]-p1[0]
    c = p1[0]*p2[1]-p2[0]*p1[1]
    line = [a,b,c]
    return line
def LineFromState(state, block_type):
    x = state[0]
    y = state[1]
    w = state[2]
    h = state[3]
    theta = state[4]/180*pi

    pts = RotatePts(state, block_type)   
    print(block_type) 
    if block_type in ("metalrectangle", "woodrectangle", "ground"):        
        # generally, the direction of width
        p1 = pts[1]
        p2 = pts[2]
        line = LineFrom2pts(p1,p2)
    elif block_type == "metalrtriangle" or block_type == "woodrtriangle":
        # generally, the direction of hypotenuse
        p1 = pts[0]
        p2 = pts[1]
        line = LineFrom2pts(p1,p2)
    return line

def GetDistancePt2Block(pt, state, obj_type):
    if obj_type == "ground":
        pts = RotatePts(state, obj_type)
        dist_min = 500
        for i in range(len(pts)):
            dist_temp = GetDistance(pt, pts[i])
            if dist_temp < dist_min:
                dist_min = dist_temp
                idx_min = i
        # get 2 line
        line1 = LineFrom2pts(pts[idx_min],pts[idx_min-1])
        if idx_min+1 >= len(pts):
            line2 = LineFrom2pts(pts[idx_min],pts[idx_min+1-len(pts)])
        else:
            line2 = LineFrom2pts(pts[idx_min],pts[idx_min+1])
        # get 2 perpendicular feet
        _, p1 = GetFootPerpendicular(pt, line1)
        _, p2 = GetFootPerpendicular(pt, line2)
        # calculate the angle
        if (pts[idx_min][0]-p1[0])*(pts[idx_min-1][0]-p1[0])+(pts[idx_min][1]-p1[1])*(pts[idx_min-1][1]-p1[1])<0:
            dist_min = GetDistance(pt, p1)
            pt_min = p1
        elif (pts[idx_min][0]-p2[0])*(pts[(idx_min+1)%len(pts)][0]-p2[0])+(pts[idx_min][1]-p2[1])*(pts[(idx_min+1)%len(pts)][1]-p2[1])<0:
            dist_min = GetDistance(pt, p2)
            pt_min = p2
        else:
            dist_min = GetDistance(pt, pts[idx_min])
            pt_min = pts[idx_min] 
    else:
        dist_min = 500
        pt_min = [0,0]

    return dist_min, pt_min

def GetDistanceBlock2Block(obj1_state, obj2_state, obj1_type, obj2_type):
    pts1 = RotatePts(obj1_state, obj1_type)
    pts2 = RotatePts(obj2_state, obj2_type)
    dist_min = 500    
    for pt1 in pts1:
        dist, pt_min = GetDistancePt2Block(pt1, obj2_state, obj2_type)
        if dist < dist_min:
            dist_min = dist
            dist_vector = [pt_min[0]-pt1[0],pt_min[1]-pt1[1]]
    for pt2 in pts2:
        dist, pt_min = GetDistancePt2Block(pt2, obj1_state, obj1_type)
        if dist < dist_min:
            dist_min = dist
            dist_vector = [pt_min[0]-pt2[0],pt_min[1]-pt2[1]]
    return dist_vector, dist

def GetFootPerpendicular(pt, line):
    a = line[0]
    b = line[1]
    c = line[2]
    dist = abs(a*pt[0]+b*pt[1]+c)/sqrt(a**2+b**2)
    x = (b**2*pt[0]-a*b*pt[1]-a*c)/(a**2+b**2)
    y = (-b*a*pt[0]+a**2*pt[1]-b*c)/(a**2+b**2)
    pt_foot = [x,y]
    return dist, pt_foot

def RotatePts(state, block_type):
    # [x,y,w,h,rot]
    x = state[0]
    y = state[1]
    w = state[2]
    h = state[3]    
    theta = state[4]/180*pi  
    if block_type == "metalrtriangle" or block_type == "woodrtriangle":  
        alpha = pi/4 # isosceles triangle
        cx = x + w/2
        cy = y + h/2
        l = sqrt(w**2+h**2)/2
        pts = [[cx+l*cos(theta+alpha),cy+l*sin(theta+alpha)],[cx-l*cos(theta+alpha),cy-l*sin(theta+alpha)],
        [cx-l*cos(theta-alpha),cy-l*sin(theta-alpha)]] # [p1,p2,p3] = [[x+w,y+h],p2,p3], counter clockwise
    else: 
        alpha = atan(h/w) # we can replace it with using cos(a+b) = cos(a)cos(b)-sin(a)sin(b)
        cx = x + w/2
        cy = y + h/2
        l = sqrt(w**2+h**2)/2
        pts = [[cx+l*cos(theta+alpha),cy+l*sin(theta+alpha)],[cx+l*cos(theta-alpha),cy+l*sin(theta-alpha)],
        [cx-l*cos(theta+alpha),cy-l*sin(theta+alpha)],[cx-l*cos(theta-alpha),cy-l*sin(theta-alpha)]] # [p1,p2,p3,p4] = [[x+w,y+h],p2,p3,p4], counterclockwise
    return pts

def LineInequality(p1,p2,ptest):
    # check where the ptest is about the line p1p2
    a = p1[0]
    b = p1[1]
    c = p2[0]
    d = p2[1]
    x = ptest[0]
    y = ptest[1]
    if c-a != 0:
        ineq = y - (d-b)/(c-a)*x-(b*c-a*d)/(c-a)
    else:
        ineq = a-x
    return ineq
def CheckIntersect(p1,p2,q1,q2):
    # Check if the line p1p2 and the line q1q2 intersect    
    eq1 = LineInequality(p1,p2,q1)*LineInequality(p1,p2,q2)
    eq2 = LineInequality(q1,q2,p1)*LineInequality(q1,q2,p2)
    if (eq1<=0 and eq2<0) or (eq1<0 and eq2<=0):
        bool_intersect = True
    else:
        bool_intersect = False
    return bool_intersect
def GetIntersectPt(p1,p2,q1,q2):
    # only when the intersection exists
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    x3 = q1[0]
    y3 = q1[1]
    x4 = q2[0]
    y4 = q2[1]
    den = (y4-y3)*(x2-x1)-(x4-x3)*(y2-y1)
    t_num = (x4-x3)*(y1-y3)-(y4-y3)*(x1-x3)
    s_num = (x2-x1)*(y1-y3)-(y2-y1)*(x1-x3) 

    t = t_num/den
    s = s_num/den
    x = x1 + t*(x2-x1)
    y = y1 + t*(y2-y1)
    pt_intersect = [x,y]
    return pt_intersect
def CheckInside(point_list, ptest):
    '''
    Check if the point is inside or outside of covex polygon
    '''
    # point_list [[p1,p2,...]] = [[x1,y1],[x2,y2],....]
    n_pt = len(point_list)
    x_cm = 0
    y_cm = 0
    bool_inside = True
    for pt in point_list:
        x_cm = x_cm + pt[0]
        y_cm = y_cm + pt[1]
    # Find the center of polygon
    x_cm = x_cm/n_pt
    y_cm = y_cm/n_pt
    pcm = [x_cm,y_cm]
    ineq_compare = []
    for i in range(n_pt-1):
        # choose 2 elements in a row from the list
        p1 = point_list[i]
        p2 = point_list[i+1]
        ineq_compare.append(LineInequality(p1,p2,pcm))
        if LineInequality(p1,p2,pcm)*LineInequality(p1,p2,ptest) < 0:
            bool_inside = False
    # the segment between the last point and the first point
    p1 = point_list[n_pt-1]
    p2 = point_list[0]
    ineq_compare.append(LineInequality(p1,p2,pcm))
    if LineInequality(p1,p2,pcm)*LineInequality(p1,p2,ptest) < 0:
        bool_inside = False
    return bool_inside

def CheckIntersectPolygon(point_list, p1, p2):
    # Check if the line segment intersect the polygon or not
    bool_intersect = False
    line_intersect = []
    pts_intersect = []
    n_pt = len(point_list)
    for i in range(n_pt-1):
        q1 = point_list[i]
        q2 = point_list[i+1]
        if CheckIntersect(p1,p2,q1,q2):
            bool_intersect = True
            line_intersect.append([q1,q2])
            pts_intersect.append(GetIntersectPt(p1,p2,q1,q2))
    # the segment between the last point and the first point
    q1 = point_list[n_pt-1]
    q2 = point_list[0]
    if CheckIntersect(p1,p2,q1,q2):
        bool_intersect = True
        line_intersect.append([q1,q2])
        pts_intersect.append(GetIntersectPt(p1,p2,q1,q2))
    return bool_intersect, line_intersect, pts_intersect

def ClosestNeighbor(point_list, ptest, k):
    # Choose k nearest points from ptest
    n_pt = len(point_list)
    dist = []
    for i in range(n_pt):
        ptarget = point_list[i]
        dist.append([ptarget[0], ptarget[1], sqrt((ptarget[0]-ptest[0])**2+(ptarget[1]-ptest[1])**2)])
    dist.sort(key = lambda x: x[2]) # descendingly by the 3rd element(distance)
    knnlist = [dist[i][0:2] for i in range(k+1)]
    # remove itself
    knnlist.remove(knnlist[0])
    return knnlist

# Example about How to use
if __name__ == "__main__":
    plist = [[0,0],[0,1],[1,1],[1,0]] # it should be in order 
    r1 = CheckInside(plist, [2,0])
    r2 = CheckInside(plist, [0.5,0.2])
    print(r1,r2)
    ptest = [1.1,1.2]
    knn = ClosestNeighbor(plist, ptest, 3)
    print(knn)

    a = CheckIntersect([300,220],[375,220],[367.5844354960768, 219.05725880568156],[366.11887974599153, 221.9906774753703])
    print(a)
    print(GetIntersectPt([402.3308708656465, 181.99014409829647], [397.3171275100915, 185.25499744196844],[473.3124530398655, 157.0730365071494], [398.1370433769928, 184.43464797320289]))
    print(GetIntersectPt([402.3308708656465, 181.99014409829647], [397.3171275100915, 185.25499744196844],[406.6875469601345, 207.9269634928506], [398.1370433769928, 184.43464797320289]))
    print(CheckIntersectPolygon([[473.3124530398655, 157.0730365071494], [398.1370433769928, 184.43464797320289],[406.6875469601345, 207.9269634928506]],[402.3308708656465, 181.99014409829647], [397.3171275100915, 185.25499744196844]))
    pre = [402.3308708656465, 181.99014409829647]
    post = [397.3171275100915, 185.25499744196844]
    ptlist = [[473.3124530398655, 157.0730365071494], [398.1370433769928, 184.43464797320289],[406.6875469601345, 207.9269634928506]]
    _, _, intspt = CheckIntersectPolygon(ptlist, pre, post)
    print(GetIntersectPt(pre, post, ptlist[0], ptlist[1]))
    print(GetIntersectPt(pre, post, ptlist[2], ptlist[1]))
    plt.axis([0-1,500+1, 500+1,0-1])
    plt.scatter(pre[0], pre[1], c = 'r', s = 3)
    plt.scatter(post[0], post[1], c = 'r', s = 3)
    plt.scatter(intspt[0][0],intspt[0][1], c = 'g', s = 3)
    plt.scatter(intspt[1][0], intspt[1][1], c = 'b', s = 3)
    plt.plot([ptlist[0][0], ptlist[1][0], ptlist[2][0]],[ptlist[0][1], ptlist[1][1], ptlist[2][1]], c = 'k')
    plt.show()