# RPM Class
import random
from math import sqrt, cos, sin, pi, atan
if __name__ == "__main__":
    from utils import CheckInside, CheckIntersect, CheckIntersectPolygon, ClosestNeighbor
else:
    from .utils import CheckInside, CheckIntersect, CheckIntersectPolygon, ClosestNeighbor

'''
Args : 
Returns : 
'''
class PRM:
    def __init__(self, map_size, obs_slist, p_start, p_end):
        self.map_size = map_size # [xmin=0, xmax, ymin=80, ymax]        
        self.obs_slist = obs_slist # [x,y,width,height,rotation] N by 5
        self.p_start = p_start # [x,y]
        self.p_end = p_end
        self.path = [p_start]
        self.p_sample = []
    def ObstacleEnv(self, ptest = [0,0]):
        # Check if ptest is inside the obstacle region
        # Assume the rectangle
        free_bool = True
        obs_region = []
        if type(self.obs_slist[0]) == list:
            for s_obs in self.obs_slist:
                w = s_obs[2]
                h = s_obs[3]
                theta = s_obs[4]/180*pi
                alpha = atan(h/w) # we can replace it with using cos(a+b) = cos(a)cos(b)-sin(a)sin(b)
                cx = s_obs[0] + w/2
                cy = s_obs[1] + h/2
                l = sqrt(w**2+h**2)/2
                obs_region.append([[cx+l*cos(theta+alpha),cy+l*sin(theta+alpha)],[cx+l*cos(theta-alpha),cy+l*sin(theta-alpha)],
                [cx-l*cos(theta+alpha),cy-l*sin(theta+alpha)],[cx-l*cos(theta-alpha),cy-l*sin(theta-alpha)]]) # [p1,p2,p3,p4] = [[x,y],p2,p3,p4], counter clockwise
        else:
            s_obs = self.obs_slist
            w = s_obs[2]
            h = s_obs[3]
            theta = s_obs[4]/180*pi
            alpha = atan(h/w) # we can replace it with using cos(a+b) = cos(a)cos(b)-sin(a)sin(b)
            cx = s_obs[0] + w/2
            cy = s_obs[1] + h/2
            l = sqrt(w**2+h**2)/2
            obs_region.append([[cx+l*cos(theta+alpha),cy+l*sin(theta+alpha)],[cx+l*cos(theta-alpha),cy+l*sin(theta-alpha)],
            [cx-l*cos(theta+alpha),cy-l*sin(theta+alpha)],[cx-l*cos(theta-alpha),cy-l*sin(theta-alpha)]]) # [p1,p2,p3,p4] = [[x,y],p2,p3,p4], counter clockwise
        for obs_polygon in obs_region:
            if CheckInside(obs_polygon,ptest):
                free_bool = False
        return free_bool, obs_region
    def Sampling(self, num_samples):
        '''
        Randomly choose the samples
        connect p_start and p_end / select the samples around that line (average is the line) 
        '''
        xmin = 0
        ymin = 0
        xmax = self.map_size[1]
        ymax = self.map_size[3]        
        num = 0
        while num<num_samples:
            # sampling randomly
            xtemp = random.randint(xmin, xmax)
            ytemp = random.randint(ymin, ymax)
            p_temp = [xtemp, ytemp]
            # sampling around the line + sampling randomly
            ####
            free_bool, obs_region = self.ObstacleEnv(p_temp)
            if free_bool and not ([xtemp,ytemp] in self.p_sample) and p_temp != self.p_start and p_temp != self.p_end:
                self.p_sample.append([xtemp,ytemp])
                num += 1         
        return self.p_sample  
    def Addstartend(self):
        # Add start/end point
        self.p_sample.append(self.p_start)
        self.p_sample.append(self.p_end)
    def ConnectDots(self, k):        
        n_smp = len(self.p_sample)
        connect_line = []
        free_bool, obs_region = self.ObstacleEnv()
        # k closest neighbors
        for pt in self.p_sample:
            knn = ClosestNeighbor(self.p_sample,pt,k)
            for q in knn:
                bool_linefree = True
                for obs_polygon in obs_region:
                    if CheckIntersectPolygon(obs_polygon,pt,q):
                        bool_linefree = False      
                if bool_linefree and not ([q,pt] in connect_line):
                    connect_line.append([pt,q]) # connect_line = [[p1,q1],[p2,q2],...] = [[[1,2],[2,3]],[[2,5],[3,3]],...]
        return connect_line
    def FindConnectedNeighbor(self, connect_line, pt):
        neighbors = []
        for line in connect_line:
            if pt == line[0]:
                neighbors.append(line[1])
            elif pt == line[1]:
                neighbors.append(line[0])
        return neighbors
    def Astar(self, connect_line):
        frontier = []
        priority = []
        cost_so_far = {}
        came_from = {} # when next is given, we can know where the next should be from
        path = [] # all of the state we explored
        shortest = []
        # start 
        frontier.append(self.p_start)     
        priority.append(0)   
        cost_so_far["{}".format(self.p_start)] = 0
        came_from["{}".format(self.p_start)] = None

        while (not frontier) == False:
            # Choose the highest priority of frontier list as the current
            #print(frontier)
            current = frontier[0]            
            frontier.remove(current)
            priority.remove(priority[0])
            # Find the path
            if current == self.p_end:
                path.append(current)
                break
            for nexts in self.FindConnectedNeighbor(connect_line,current):
                new_cost = cost_so_far["{}".format(current)] + sqrt((current[0]-nexts[0])**2+(current[1]-nexts[1])**2)                
                # Add next into the frontier or change cost
                # We should be careful of path and froniter are different (path : explored, frontier : check only cost)
                if (nexts not in path and nexts not in frontier) or new_cost < cost_so_far["{}".format(nexts)]:
                    cost_so_far["{}".format(nexts)] = new_cost 
                    # Decide the heuristic cost for priority
                    cost_p = new_cost + abs(self.p_end[0]-nexts[0])+abs(self.p_end[1]-nexts[1])
                    # Check next's priority
                    i = 0
                    for cost_temp in priority:
                        if cost_temp >= cost_p:
                            i = priority.index(cost_temp)
                            break
                        elif priority.index(cost_temp) == len(priority)-1:
                            i = len(priority)
                    # Check if this next was the previous frontier
                    if nexts not in path:
                        frontier.insert(i,nexts)
                        priority.insert(i,cost_p)
                    came_from["{}".format(nexts)] = current
            path.append(current)
        #print("cost so far : {}".format(cost_so_far))
        # Find the shortest path
        pre = self.p_end
        shortest = [self.p_end]
        while pre != self.p_start:
            pre = came_from["{}".format(pre)]
            shortest.append(pre)
        return path, shortest, came_from
    def ExcludePoints(self, point_list, k):
        '''
        point_list : What we wanna exclude
        k : k-nearest points are also excluded
        '''
        # Exclude the point_list from the samples set
        for i in range(len(point_list)):           
            self.p_sample.remove(point_list[i])
        # Exclude the closest point list from the sample set
        for i in range(len(point_list)):
            knn = ClosestNeighbor(self.p_sample,point_list[i],k)
            for j in range(k):
                self.p_sample.remove(knn[j])
        return self.p_sample        
# Example about How to use

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    map_size = [0,500,0,500]
    obs_list = [[50,50,50,125,0],[150,200,100,20,0]]
    p_start = [10,10]
    p_end = [200,400]
    prm = PRM(map_size, obs_list, p_start, p_end)
    num_samples = 100
    sampling_list = prm.Sampling(num_samples)
    #print(sampling_list)
    prm.Addstartend()
    k = 4
    connect_line = prm.ConnectDots(k)
    #print("connect line: {}".format(connect_line))
    explored_path, shortest_path, came_from = prm.Astar(connect_line)
    print("explored_path : {}".format(explored_path))
    print("shortest path : {}".format(shortest_path))
    # path planning figure
    fig, ax = plt.subplots()
    ax.set(xlim = (0-1,map_size[1]+1), ylim = (map_size[3]+1,0-1))

    xpath = [shortest_path[i][0] for i in range(len(shortest_path))]
    ypath = [shortest_path[i][1] for i in range(len(shortest_path))]
    xsample = [sampling_list[i][0] for i in range(len(sampling_list))]
    ysample = [sampling_list[i][1] for i in range(len(sampling_list))]

    for j in range(len(connect_line)):
        x = [connect_line[j][0][0],connect_line[j][1][0]]
        y = [connect_line[j][0][1],connect_line[j][1][1]]
        ax.plot(x,y, color = 'k')
    ax.plot(xpath,ypath, c = 'b')
    ax.scatter(xsample,ysample, s = 10, c = 'r')
    plt.show()