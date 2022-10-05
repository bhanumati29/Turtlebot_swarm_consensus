#! /usr/bin/env python

from tkinter.tix import X_REGION
from wave import Wave_write
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

def newOdom1(msg):
    global x1, y1, theta1
    x1 = msg.transform.translation.x
    y1 = msg.transform.translation.y
    rot_q = msg.transform.rotation
    (roll, pitch, theta1) = euler_from_quaternion(rot_q.x, rot_q.y, rot_q.z, rot_q.w)

global vv, Ww
vv, ww = 0.0, 0.0

def js_callback(j_msg):
    global axiss, buttons, mode
    axiss = j_msg.axes
    buttons = j_msg.buttons
    global vv, ww
    vv = axiss[1]*0.22*0.75
    ww = axiss[2]*0.75

rospy.init_node("js_confne_ws")

sub1 = rospy.Subscriber("/vicon/TB1/TB1", TransformStamped, newOdom1)

sub = rospy.Subscriber("/joy", Joy, js_callback)

pub1 = rospy.Publisher("/TB1/cmd_vel", Twist, queue_size = 10)

speed1 = Twist()

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
	global x1, y1, theta1

	global t_now
	global vv, ww

	v1, w1 = vv, ww

	r1_x = 1.4849704495
	r1_y = 0.5468573723

	r2_x = -1.096957838
	r2_y = 0.4887769998

	r3_x = -1.183408936
	r3_y = -0.883100281

	r4_x = 1.4036759455
	r4_y = -0.971494976

	r0 = 1.0
	rd = 0.75
	r = math.sqrt(x1**2 + y1**2)

	V_max = 0.22*0.75
	dr_dt = 0.0
	if(r<rd):
		dr_dt = 0.0
	elif((r>=rd) and (r<r0)):
		dr_dt = -V_max*(r-rd)/(r0-rd)
	else:
		dr_dt = -V_max


	V,W = 0.0, 0.0

	alpha = math.atan2(y1, x1)
	Vx_reg = dr_dt*math.cos(alpha)
	Vy_reg = dr_dt*math.sin(alpha)

	Vx_com = v1*math.cos(theta1)
	Vy_com = v1*math.sin(theta1)

	Vx = Vx_reg + Vx_com
	Vy = Vy_reg + Vy_com

	# delta_theta = wrap_pi(math.atan2(Vy, Vx) - theta1)
	V = Vx*math.cos(theta1) + Vy*math.sin(theta1)

	if(r>rd):
		beta = alpha - theta1
		beta = wrap_pi_2(wrap_pi(beta))
		Kw = 2.84*0.5/(90.0*math.pi/180.0)
		W = Kw*beta
	else:
		W = w1

	# if(abs(V)<0.02):
	# 	V = 0.0
	
	v_max = 0.22*0.75
	w_max = 2.84*0.5 #162.72*math.pi/180.0 # 2.83999975885 rad/sec

	v1 = limit(V, -v_max, v_max)
	w1 = limit(W, -w_max, w_max)

	print(round(t_now,2), round(r,2), round(V,2))

	return v1, w1

def get_traj(t):
	x, y, th, vx, vy, vth = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

	Ax = 1.0
	Ay = 0.25
	T = 40.0
	w = 2.0*math.pi/T
	wx = w
	wy = 2.0*wx
	x = Ax*math.sin(wx*t)
	y = Ay*math.sin(wy*t)
	vx = Ax*wx*math.cos(wx*t)
	vy = Ay*wy*math.cos(wy*t)
	th = math.atan2(vy, vx)
	ax = -Ax*(wx**2)*math.sin(wx*t)
	ay = -Ay*(wy**2)*math.sin(wy*t)
	vth = (vx*ay - vy*ax) / ((vx**2) + (vy**2))
	v_r = math.sqrt(vx**2 + vy**2)
	w_r = vth

	return x, y, th, vx, vy, vth, v_r, w_r

def traj_tracking():
	K_x  = 0.1
	K_y  = 0.08**2
	K_th = 2.0*math.sqrt(K_y)

	global x1, y1, theta1, t_now
	x_c, y_c, th_c = x1, y1, theta1
	x_0, y_0, th_0, V_x, V_y, V_th, V_r, W_r = get_traj(t_now)

	p_r = np.array([x_0, y_0, th_0])
	p_c = np.array([x_c, y_c, th_c])
	T_e = np.matrix([[math.cos(th_c), math.sin(th_c), 0.0], [-math.sin(th_c), math.cos(th_c), 0.0], [0.0, 0.0, 1.0]])
	p_e = np.matmul(T_e,(p_r - p_c))
	x_e, y_e, th_e = p_e[0,0], p_e[0,1], p_e[0,2]
	v = V_r*math.cos(th_e) + K_x*x_e
	w = W_r + V_r*(K_y*y_e + K_th*math.sin(th_e))

	print('---')
	print(v,w)
	v,w = 0.0, 0.0

	return v, w

r = rospy.Rate(10)

while not rospy.is_shutdown():
	if(start_var==True):
		start_var = False
		t_init = time.time()
	t_now = time.time() - t_init

	try:
		# v1, w1 = cal_vw(t_now)
		v1, w1 = traj_tracking()

		speed1.linear.x  = v1
		speed1.angular.z = w1

		pub1.publish(speed1)

		# print(t_now, t_now, v1, w1, v2, w2, v3, w3, v4, w4)		
		# print('going!.........', mode)

	except:
		print('error!')
		pass
	r.sleep()