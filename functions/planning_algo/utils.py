# utils
import numpy as np
from math import sqrt

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
    if eq1<=0 and eq2<=0:
        bool_intersect = True
    else:
        bool_intersect = False
    return bool_intersect
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
    n_pt = len(point_list)
    for i in range(n_pt-1):
        q1 = point_list[i]
        q2 = point_list[i+1]
        if CheckIntersect(p1,p2,q1,q2):
            bool_intersect = True
    return bool_intersect
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