# utils
from scipy.optimize import least_squares
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt

g = 10	
def Parabola(a, x):
	# x should be np.array	
	vx = a[0]
	vy = a[1]
	y = vy/vx*x + g/2/(vx**2)*x**2
	return y
def ObjectiveFun(a, x, y):
	vx = a[0]
	vy = a[1]
	return Parabola(a,x)-y

def FindParabola(point_list):
	a0 = np.array([1,0])
	x_train = np.array([point_list[i][0] for i in range(len(point_list))])
	y_train = np.array([point_list[i][1] for i in range(len(point_list))])
	res_lsq = least_squares(ObjectiveFun, a0, args=(x_train,y_train))
	#res_lsq = least_squares(ObjectiveFun, a0, loss='cauchy', f_scale=0.1, args=(x_train,y_train))
	print(res_lsq.x)
	print(res_lsq.cost)
	if x_train[0] < x_train[1]:
		x_query = np.arange(x_train[0],x_train[-1]+0.25, 0.25)
	else:
		x_query = np.arange(x_train[0],x_train[-1]-0.25, -0.25)
	y_lsq = Parabola(res_lsq.x, x_query)
	return x_query, y_lsq
def FindParabola2pt(pt1, pt2):
	# Solve a,b for y = ax + bx^2	
	x1 = pt1[0]
	y1 = pt1[1]
	x2 = pt2[0]
	y2 = pt2[1]
	a = (x2**2*y1-x1**2*y2)/(x1*x2**2-x2*x1**2)
	b = (-x2*y1+x1*y2)/(x1*x2**2-x2*x1**2)
	print(a,b)
	# vx, vy at pt1
	vx = sqrt(g/2/b)
	vy = a*vx
	# vx', vy' at pt2
	vy2 = vy+g*(x2-x1)/vx
	step = (x2-x1)/10
	x_query = np.arange(x1, x2+step, step)
	y_query = a*x_query + b*x_query**2
	return x_query, y_query, vx, vy, vy2

if __name__ == "__main__":
	#pt_list = [[0,0],[1,1],[2,4],[3,9],[4,16]]
	pt_list = [[1,2],[4,5]]
	x_pt = [pt_list[i][0] for i in range(len(pt_list))]
	y_pt = [pt_list[i][1] for i in range(len(pt_list))]
	x, y = FindParabola(pt_list)
	x2, y2, _, _, _ = FindParabola2pt(pt_list[0],pt_list[1])
	plt.figure()
	plt.axis([-1,20,20,-1])
	plt.plot(x, y, 'b', x2, y2, 'r')
	plt.scatter(x_pt, y_pt, c = 'g')
	plt.xlabel("x")
	plt.ylabel("y")
	plt.title("trajectory")
	plt.grid(True)
	plt.show()