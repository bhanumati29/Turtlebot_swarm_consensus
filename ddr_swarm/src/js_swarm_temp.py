#! /usr/bin/env python

from matplotlib.pyplot import axes, axis
import rospy
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Joy 
import math
import time

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

mode = 'translation'
mode_list = ['translation', 'rotation', 'aggregation', 'dispersion', 'homing', 'avoidance']

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

	if(abs(axiss[2]) > 0.5):
		mode = 'rotation'
	else:
		if(axiss[5] < -0.5):
			mode = 'aggregation'
		elif(axiss[5] > 0.5):
			mode = 'dispersion'
		else:
			mode = 'translation'



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

def get_2_closest(D):
	c1, c2, c3, c4 = [], [], [], []
	for i in range(1,5):
		for j in range(1,5):
			if(i==j):
				continue
			else:
				c1.append(i)
				c2.append(j)
				c3.append(D[i-1][j-1])
				c4.append(i*j)

def fun1(t_now):
	global x1, x2, x3, x4
	global y1, y2, y3, y4
	global theta1, theta2, theta3, theta4

	d11 = math.sqrt((x1-x1)**2+(y1-y1)**2)
	d12 = math.sqrt((x1-x2)**2+(y1-y2)**2)
	d13 = math.sqrt((x1-x3)**2+(y1-y3)**2)
	d14 = math.sqrt((x1-x4)**2+(y1-y4)**2)

	d21 = math.sqrt((x2-x1)**2+(y2-y1)**2)
	d22 = math.sqrt((x2-x2)**2+(y2-y2)**2)
	d23 = math.sqrt((x2-x3)**2+(y2-y3)**2)
	d24 = math.sqrt((x2-x4)**2+(y2-y4)**2)

	d31 = math.sqrt((x3-x1)**2+(y3-y1)**2)
	d32 = math.sqrt((x3-x2)**2+(y3-y2)**2)
	d33 = math.sqrt((x3-x3)**2+(y3-y3)**2)
	d34 = math.sqrt((x3-x4)**2+(y3-y4)**2)

	d41 = math.sqrt((x4-x1)**2+(y4-y1)**2)
	d42 = math.sqrt((x4-x2)**2+(y4-y2)**2)
	d43 = math.sqrt((x4-x3)**2+(y4-y3)**2)
	d44 = math.sqrt((x4-x4)**2+(y4-y4)**2)

	D = [[d11, d12, d13, d14], [d21, d22, d23, d24], [d31, d32, d33, d34], [d41, d42, d43, d44]]

	get_D(D, i, j)

	c1, c2, c3, c4 = get_2_closest(D)

def cal_vw(t):
	global x1, x2, x3, x4
	global y1, y2, y3, y4
	global theta1, theta2, theta3, theta4
	global mode, axiss, buttons

	v1, v2, v3, v4 = 0, 0, 0, 0
	w1, w2, w3, w4 = 0, 0, 0, 0
	
	if(mode == 'rotation'):
		global mode, axiss, buttons
		k_rot = 0.2
		w1 = k_rot*axiss[2]
		w2 = k_rot*axiss[2]
		w3 = k_rot*axiss[2]
		w4 = k_rot*axiss[2]
	elif(mode == 'translation'):
		global mode, axiss, buttons
		V_x = axiss[1]
		V_y = axiss[0]
		Kv = 0.11667261889
		V_ = Kv*math.sqrt(V_x**2+V_y**2)
		alpha = math.atan2(V_y, V_x)

		if(abs(V_)<0.001):
			v1, v2, v3, v4 = 0, 0, 0, 0
			w1, w2, w3, w4 = 0, 0, 0, 0
		else:
			beta_1 = alpha - theta1
			beta_2 = alpha - theta2
			beta_3 = alpha - theta3
			beta_4 = alpha - theta4

			beta_1 = wrap_pi(beta_1)
			beta_2 = wrap_pi(beta_2)
			beta_3 = wrap_pi(beta_3)
			beta_4 = wrap_pi(beta_4)

			v1 = V_*math.cos(beta_1)
			v2 = V_*math.cos(beta_2)
			v3 = V_*math.cos(beta_3)
			v4 = V_*math.cos(beta_4)

			beta_1 = wrap_pi_2(beta_1)
			beta_2 = wrap_pi_2(beta_2)
			beta_3 = wrap_pi_2(beta_3)
			beta_4 = wrap_pi_2(beta_4)

			Kw = 1.42/(90.0*math.pi/180.0)
			w1 = Kw*beta_1
			w2 = Kw*beta_2
			w3 = Kw*beta_3
			w4 = Kw*beta_4

	elif(mode == 'agregation'):
		pass
	elif(mode == 'dispersion'):
		pass
	elif(mode == 'homing'):
		pass

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

def limit(x, x_min, x_max):
	if x < x_min:
		return x_min
	elif x > x_max:
		return x_max
	else:
		return x


r = rospy.Rate(10)

while not rospy.is_shutdown():
	if(start_var==True):
		start_var = False
		t_init = time.time()
	t_now = time.time() - t_init

	try:
		# v1, w1, v2, w2, v3, w3, v4, w4 = cal_vw(t_now)
		v1, w1, v2, w2, v3, w3, v4, w4 = fun1(t_now)

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