#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Joy 
import math
import time
import numpy as np

def euler_from_quaternion(q1, q2, q3, q0):
	phi, th, psi = 0, 0, 0
	psi = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2**2+q3**2))
	return phi, th, psi

global t_now, t_init
global start_var
t_now = time.time()
t_init = time.time()
start_var = True

global x1, x2, x3, x4
global y1, y2, y3, y4
global theta1, theta2, theta3, theta4
global buttons, axiss, mode, mode_list

x1, x2, x3, x4 = 0, 0, 0, 0
y1, y2, y3, y4 = 0, 0, 0, 0
theta1, theta2, theta3, theta4 = 0, 0, 0, 0
axiss = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

def limit(x, x_min, x_max):
	if x < x_min:
		return x_min
	elif x > x_max:
		return x_max
	else:
		return x

mode = 'translation'
mode_list = ['translation', 'rotation', 'aggregation', 'dispersion', 'homing', 'avoidance','consensus']

def newOdom1(msg):
    global x1, y1, theta1
    x1 = msg.transform.translation.x
    y1 = msg.transform.translation.y
    rot_q = msg.transform.rotation
    (roll, pitch, theta1) = euler_from_quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)
    
def newOdom2(msg):
    global x2, y2, theta2
    x2 = msg.transform.translation.x
    y2 = msg.transform.translation.y
    rot_q = msg.transform.rotation
    (roll, pitch, theta2) = euler_from_quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)

def newOdom3(msg):
    global x3, y3, theta3
    x3 = msg.transform.translation.x
    y3 = msg.transform.translation.y
    rot_q = msg.transform.rotation
    (roll, pitch, theta3) = euler_from_quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)

def newOdom4(msg):
    global x4, y4, theta4
    x4 = msg.transform.translation.x
    y4 = msg.transform.translation.y
    rot_q = msg.transform.rotation
    (roll, pitch, theta4) = euler_from_quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)

def js_callback(j_msg):
	global axiss, buttons, mode
	axiss = j_msg.axes
	buttons = j_msg.buttons

	# if(abs(axiss[2]) > 0.5):
	# 	mode = 'rotation'
	# else:
	# 	if(axiss[5] < -0.5):
	# 		mode = 'aggregation'
	# 	elif(axiss[5] > 0.5):
	# 		mode = 'dispersion'
	# 	else:
	# 		mode = 'translation'
	mode = 'dispersion'

def get_farthest_robot(D):
	n_row = D.shape[0]
	n_col = D.shape[1]
	farthest_arr = np.zeros(n_row, dtype=np.int)
	for i in range(n_row):
		max_dist = 0
		max_idx = 0
		for j in range(n_col):
			if(D[i,j] >= max_dist): 
				max_dist = D[i,j]
				max_idx = j
		farthest_arr[i] = max_idx
	return farthest_arr

def get_nearest_robot(D):
	n_row = D.shape[0]
	n_col = D.shape[1]
	nearest_arr = np.zeros(n_row, dtype=np.int)
	for i in range(n_row):
		min_dist = 99999.9
		min_idx = 0
		for j in range(n_col):
			if (i == j):
				pass
			else:
				if(D[i,j] < min_dist): 
					min_dist = D[i,j]
					min_idx = j
		nearest_arr[i] = min_idx
	return nearest_arr

rospy.init_node("speed_controller")

sub1 = rospy.Subscriber("/vicon/TB1/TB1", TransformStamped, newOdom1)
sub2 = rospy.Subscriber("/vicon/TB2/TB2", TransformStamped, newOdom2)
sub3 = rospy.Subscriber("/vicon/TB3/TB3", TransformStamped, newOdom3)
sub4 = rospy.Subscriber("/vicon/TB4/TB4", TransformStamped, newOdom4)

sub = rospy.Subscriber("/joy", Joy, js_callback)

pub1 = rospy.Publisher("/TB1/cmd_vel", Twist, queue_size = 3)
pub2 = rospy.Publisher("/TB2/cmd_vel", Twist, queue_size = 3)
pub3 = rospy.Publisher("/TB3/cmd_vel", Twist, queue_size = 3)
pub4 = rospy.Publisher("/TB4/cmd_vel", Twist, queue_size = 3)

speed1 = Twist()
speed2 = Twist()
speed3 = Twist()
speed4 = Twist()
speed5 = Twist()

def wrap_pi(x):
	if(x>math.pi):
		return x-2*math.pi
	elif(x<=-math.pi):
		return x+2*math.pi
	else:
		return x

def wrap_pi_2(x):
	if(x>math.pi/2.0):
		return x-math.pi
	elif(x<=-math.pi/2.0):
		return x+math.pi
	else:
		return x

def cal_vw(t):
    global x1, x2, x3, x4
    global y1, y2, y3, y4
    global theta1, theta2, theta3, theta4
    global mode, axiss, buttons

    v1, v2, v3, v4 = 0, 0, 0, 0
    w1, w2, w3, w4 = 0, 0, 0, 0

    # -----------------------------------------------------------------------------------------------------------------
    # aggregation
    x = np.array([x1, x2, x3, x4])
    y = np.array([y1, y2, y3, y4])
    th = np.array([theta1, theta2, theta3, theta4])

    D = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
    for i in range(0,4):
        for j in range (0,4):
            D[i][j] = math.sqrt((x[i]-x[j])**2 + (y[i]-y[j])**2)

    D = np.matrix(D)
    farthest_robot = get_farthest_robot(D)

    th_da = np.zeros(4,dtype=float)
    for i in range(0,4):
        x_j = x[farthest_robot[i]]
        y_j = y[farthest_robot[i]]
        x_a = (x_j + x[i])/2.0
        y_a = (y_j + y[i])/2.0
        th_da[i] = math.atan2(y_a-y[i], x_a-x[i])

    # -----------------------------------------------------------------------------------------------------------------
    # dispersion
    x = np.array([x1, x2, x3, x4])
    y = np.array([y1, y2, y3, y4])
    th = np.array([theta1, theta2, theta3, theta4])
    
    D = [[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]
    for i in range(0,4):
        for j in range (0,4):
            D[i][j] = math.sqrt((x[i]-x[j])**2 + (y[i]-y[j])**2)

    D = np.matrix(D)
    nearest_robot = get_nearest_robot(D)

    th_dd = np.zeros(4,dtype=float)
    for i in range(0,4):
        x_j = x[nearest_robot[i]]
        y_j = y[nearest_robot[i]]
        x_d = (x_j + x[i])/2.0
        y_d = (y_j + y[i])/2.0
        th_dd[i] = math.pi + math.atan2(y_d-y[i], x_d-x[i])

    # -----------------------------------------------------------------------------------------------------------------
	# homing
    x_h = -1.5
    y_h = -1.2
    th_dh = np.zeros(4,dtype=float)
    th_dh[0] = math.atan2(y_h-y1, x_h-x1)
    th_dh[1] = math.atan2(y_h-y2, x_h-x2)
    th_dh[2] = math.atan2(y_h-y3, x_h-x3)
    th_dh[3] = math.atan2(y_h-y4, x_h-x4)

    # -----------------------------------------------------------------------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------
    w_h = 0.0
    w_a = 0.0
    w_d = 1.0

    th_ib = w_h*th_dh + w_a*th_da + w_d*th_dd
    # -----------------------------------------------------------------------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------

    # -----------------------------------------------------------------------------------------------------------------
	# consensus
    th_ic = np.zeros(4,dtype=float)
    th_1c = (theta1 + theta2 + theta3 + theta4)/4.0
    th_2c = (theta1 + theta2 + theta3 + theta4)/4.0
    th_3c = (theta1 + theta2 + theta3 + theta4)/4.0
    th_4c = (theta1 + theta2 + theta3 + theta4)/4.0
    th_ic = np.array([th_1c, th_2c, th_3c, th_4c])

    # -----------------------------------------------------------------------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------
    w_c = 1.0
    th_id = w_c*th_ic + (1.0-w_c)*th_ib

    beta = th_id - th
    for i in range(0,4):
        beta[i] = wrap_pi(beta[i])
    
    Kw = 1.42/(90.0*math.pi/180.0)
    w = Kw*beta
    w1 = w[0]
    w2 = w[1]
    w3 = w[2]
    w4 = w[3]

    # v1, v2, v3, v4 = 0.025, 0.025, 0.025, 0.025
    
    v_max = 0.22*0.75
    w_max = 2.84*0.5 #162.72*math.pi/180.0 # 2.83999975885 rad/sec

    v1 = limit(v1, -v_max, v_max)
    v2 = limit(v2, -v_max, v_max)
    v3 = limit(v3, -v_max, v_max)
    v4 = limit(v4, -v_max, v_max)

    w1 = limit(w1, -w_max, w_max)
    w2 = limit(w2, -w_max, w_max)
    w3 = limit(w3, -w_max, w_max)
    w4 = limit(w4, -w_max, w_max)

    return v1, w1, v2, w2, v3, w3, v4, w4


r = rospy.Rate(10)

while not rospy.is_shutdown():
	if(start_var==True):
		start_var = False
		t_init = time.time()
	t_now = time.time() - t_init

	try:
		v1, w1, v2, w2, v3, w3, v4, w4 = cal_vw(t_now)

		speed1.linear.x  = v1
		speed1.angular.z = w1

		speed2.linear.x  = v2
		speed2.angular.z = w2

		speed3.linear.x  = v3
		speed3.angular.z = w3

		speed4.linear.x  = v4
		speed4.angular.z = w4

		pub1.publish(speed1)
		pub2.publish(speed2)
		pub3.publish(speed3)
		pub4.publish(speed4)

		# print(t_now, t_now, v1, w1, v2, w2, v3, w3, v4, w4)		
		# print('going!.........', mode)

	except:
		print('error!')
		pass
	r.sleep()