#!/usr/bin/env python
import roslib; roslib.load_manifest('nodes')
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import sensor_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf.transformations import euler_from_quaternion
from numpy import random
from math import *
import cv2
from cv_bridge import CvBridge, CvBridgeError
import time
import matplotlib.pyplot as plt

plt.ion()

I = 0
prev_err = 0
prev_time = 0

x_dir2 = 0
y_dir2 = 0
x_dir3 = 0
y_dir3 = 0

desirex = 0
desirey = 0

#Positions
#---[7,6,5,4,3,2,1]
x = [0,0,0,0,0,0,0]
y = [0,0,0,0,0,0,0]


const_dist = [8,6,4,2]
#------[6,5,4,3,2,1]
posex= [0,0,0,0,0,0]
posey= [0,0,0,0,0,0]
ort  = [0,0,0,0,0,0]
yaw  = [0,0,0,0,0,0]
rbt1stop = rbt2stop =  0


#World Coordinates
#----------[  [-6,-1,0.3]     [-4,-1,0.3]     [-2,-1,0.3]      [0,-1,0.3]       [2,0,0.3]      [2,-2,0.3]
#----------[[     uav6    ],[      uav5   ],[     uav4    ],[     uav3    ],[     uav2    ],[     uav1    ]]
#----------[[7,6,5,4,3,2,1],[7,6,5,4,3,2,1],[7,6,5,4,3,2,1],[7,6,5,4,3,2,1],[7,6,5,4,3,2,1],[7,6,5,4,3,2,1]]
distance = [[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0],[0,0,0,0,0,0,0]]

#----------[6,5,4,3,2,1]
altitude = [0,0,0,0,0,0]
stopflag = [0,0,0,0,0,0]
rightside = ()
leftside = ()
front= ()

#--------------------------------uav3---------------------------------------
#Position Function
def callback3(data):
	global posex, posey, ort, yaw, altitude
	posex[3] = range_sensor_data3.linear.x = data.pose.pose.position.x
	posey[3] = range_sensor_data3.linear.y = data.pose.pose.position.y
	altitude[3] = range_sensor_data3.linear.z = data.pose.pose.position.z
	ort[3] = data.pose.pose.orientation
	euler = euler_from_quaternion([ort[3].x,ort[3].y,ort[3].z,ort[3].w])
	yaw[3] = euler[2]
	range_sensor3.publish(range_sensor_data3)

#--------------------------------uav2---------------------------------------
def callback2(data):
	global posex, posey, ort, yaw, altitude
	posex[4] = range_sensor_data2.linear.x = data.pose.pose.position.x
	posey[4] = range_sensor_data2.linear.y = data.pose.pose.position.y
	altitude[4] = range_sensor_data2.linear.z = data.pose.pose.position.z
	ort[4] = data.pose.pose.orientation
	euler = euler_from_quaternion([ort[4].x,ort[4].y,ort[4].z,ort[4].w])
	yaw[4] = euler[2]
	range_sensor2.publish(range_sensor_data2)


#--------------------------------uav1---------------------------------------
def callback1(data):
	global posex, posey, ort, yaw, altitude
	posex[5] = range_sensor_data1.linear.x = data.pose.pose.position.x
	posey[5] = range_sensor_data1.linear.y = data.pose.pose.position.y
	altitude[5] = range_sensor_data1.linear.z = data.pose.pose.position.z
	ort[5] = data.pose.pose.orientation
	euler = euler_from_quaternion([ort[5].x,ort[5].y,ort[5].z,ort[5].w])
	yaw[5] = euler[2]
	range_sensor1.publish(range_sensor_data1)

#-------------------------------------------------------------------------------
#-----------------Range sensor for each uav-------------------------------------
#-------------------------------------------------------------------------------
#uav3------------------------
def uav3_range(data):
	global distance
	noise = random.normal(0,0.05,1)
	y_diff = 0
	x_diff = 0
	for i in range(6):
		z_diff = abs(altitude[i] - data.linear.z)
		y_diff = abs(posey[i] - data.linear.y)

		if (posex[i]<0 and data.linear.x>=0):
			x_diff = (posex[i]*-1) + data.linear.x
		elif(posex[i]<0 and data.linear.x<0):
			x_diff = data.linear.x - posex[i]
		elif(posex[i]>=0 and data.linear.x>0):
			x_diff = data.linear.x - posex[i]
		else:
			x_diff = posex[i] - data.linear.x
		distance[i][4] = sqrt(x_diff**2 + y_diff**2) + noise[0]

#uav2------------------------
def uav2_range(data):
	global distance
	noise = random.normal(0,0.05,1)
	y_diff = 0
	x_diff = 0
	for i in range(6):
		z_diff = abs(altitude[i] - data.linear.z)
		y_diff = abs(posey[i] - data.linear.y)
		if (posex[i]<0 and data.linear.x>=0):
			x_diff = (posex[i]*-1) + data.linear.x
		elif(posex[i]<0 and data.linear.x<0):
			x_diff = data.linear.x - posex[i]
		elif(posex[i]>=0 and data.linear.x>0):
			x_diff = data.linear.x - posex[i]
		else:
			x_diff = posex[i] - data.linear.x
		if(i!=4):
			distance[i][5] = sqrt(x_diff**2 + y_diff**2) + noise[0]

#uav1------------------------
def uav1_range(data):
	global distance
	noise = random.normal(0,0.05,1)
	y_diff = 0
	x_diff = 0
	for i in range(6):
		z_diff = abs(altitude[i] - data.linear.z)
		y_diff = abs(posey[i] - data.linear.y)
		if (posex[i]<0 and data.linear.x>=0):
			x_diff = (posex[i]*-1) + data.linear.x
		elif(posex[i]<0 and data.linear.x<0):
			x_diff = data.linear.x - posex[i]
		elif(posex[i]>=0 and data.linear.x>0):
			x_diff = data.linear.x - posex[i]
		else:
			x_diff = posex[i] - data.linear.x
		if(i!=5):
			distance[i][6] = sqrt(x_diff**2 + y_diff**2) + noise[0]

#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
#------------------------------------------------------------------------------




#-------------------------------------------------------------------------------
#---------------------------------Find Positions--------------------------------
#-------------------------------------------------------------------------------

def find_positions():
	global x,y

	x1 = 0
	y1 = 0
	x2 = 0
	y2 = distance[4][6]
	y[5] = y2
	r1 = distance[3][6]
	r2 = distance[3][5]
	try:
		xdiff = x2-x1
		ydiff = y2-y1
		dstbtwctr = sqrt(xdiff**2+ydiff**2)
		dis = (dstbtwctr**2 +r1**2-r2**2)/(2*dstbtwctr)
		x[4] = (ydiff/dstbtwctr)*sqrt(r1**2 - dis**2)
		y[4] = ydiff*dis/dstbtwctr
	except ValueError:
		pass

def find():
	x1 = 0
	y1 = 0
	x2 = 0
	y2 = distance[3][6]
	y[5] = y2
	r1 = distance[4][6]
	r2 = distance[4][4]
	try:
		xdiff = x2-x1
		ydiff = y2-y1
		dstbtwctr = sqrt(xdiff**2+ydiff**2)
		dis = (dstbtwctr**2 +r1**2-r2**2)/(2*dstbtwctr)
		x = (ydiff/dstbtwctr)*sqrt(r1**2 - dis**2)
		y = ydiff*dis/dstbtwctr
		print x,y
	except ValueError:
		pass

def desirexy(dis1,dis2):
	if ((dis1-dis2)**2<=4<=(dis1+dis2)**2):
		x1 = 0
		y1 = 0
		x2 = 0
		y2 = 2
		r1 = dis1
		r2 = dis2
		try:
			xdiff = x2-x1
			ydiff = y2-y1
			dstbtwctr = sqrt(xdiff**2+ydiff**2)
			dis = (dstbtwctr**2 +r1**2-r2**2)/(2*dstbtwctr)
			x = (ydiff/dstbtwctr)*sqrt(r1**2 - dis**2)
			y = ydiff*dis/dstbtwctr
		except ValueError:
			pass
		return ([x,y])
	else:
		return([0,0])

#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------

def uav_dir():
	rate2 = distance[4][6]
	rate3 = distance[3][6]
	twist2.linear.x = 0.1
	twist3.linear.x = 0.1
	pub2.publish(twist2)
	pub3.publish(twist3)
	time.sleep(2)
	twist2.linear.x = 0
	twist3.linear.x = 0
	pub2.publish(twist2)
	pub3.publish(twist3)
	temp_rate2 = distance[4][6]
	temp_rate3 = distance[3][6]
	if (temp_rate2<rate2):
		x2 = 1
	elif(temp_rate2>rate2):
		x2 = -1
	else:
		x2 = 0
	if (temp_rate3<rate3):
		x3 = 1
	elif(temp_rate3>rate3):
		x3 = -1
	else:
		x3 = 0
	twist2.linear.x = -0.1
	twist3.linear.x = -0.1
	pub2.publish(twist2)
	pub3.publish(twist3)
	time.sleep(2)
	twist2.linear.x = 0
	twist3.linear.x = 0
	pub2.publish(twist2)
	pub3.publish(twist3)
#---------------------------------
	rate2 = distance[4][6]
	rate3 = distance[3][6]
	twist2.linear.y = 0.1
	twist3.linear.y = 0.1
	pub2.publish(twist2)
	pub3.publish(twist3)
	time.sleep(2)
	twist2.linear.y = 0
	twist3.linear.y = 0
	pub2.publish(twist2)
	pub3.publish(twist3)
	temp_rate2 = distance[4][6]
	temp_rate3 = distance[3][6]
	if (temp_rate2<rate2):
		y2 = 1
	elif(temp_rate2>rate2):
		y2 = -1
	else:
		y2 = 0
	if (temp_rate3<rate3):
		y3 = 1
	elif(temp_rate3>rate3):
		y3 = -1
	else:
		y3 = 0
	twist2.linear.y = -0.1
	twist3.linear.y = -0.1
	pub2.publish(twist2)
	pub3.publish(twist3)
	time.sleep(2)
	twist2.linear.y = 0
	twist3.linear.y = 0
	pub2.publish(twist2)
	pub3.publish(twist3)
	return ([x2,y2,x3,y3])


def UAV2_Formation():
	global x_dir2,y_dir2,x_dir3,y_dir3
	tp = uav_dir()
	x_dir2 = tp[0]
	y_dir2 = tp[1]
	x_dir3 = tp[2]
	y_dir3 = tp[3]
	r1 = distance[4][6]
	prev = r1+1
	tmp = 0

	#x direction
	while(prev>distance[4][6] and distance[4][6]>2):
		twist2.linear.x = 0.57*x_dir2
		prev = distance[4][6]
		find_positions()
		if (x_dir3 == -1):
			frcx = (x[4]*x_dir3 + desirex)/10
		else:
			frcx = (x[4]*x_dir3 - desirex)/10

		frcy = (y[4] - desirey)/10
		if (abs(frcx)>0.4):
			frcx = 0.4*x_dir3
		elif (abs(frcx)<0.05):
			frcx = 0
		if (abs(frcy)>0.4):
			frcy = 0.4*y_dir3
		elif (abs(frcy)<0.05):
			frcy = 0
		twist3.linear.x = frcx
		twist3.linear.y = frcy
		pub2.publish(twist2)
		pub3.publish(twist3)
		time.sleep(1)
		#print x[4],y[4]
	twist2.linear.x = 0
	twist2.linear.y = 0
	pub2.publish(twist2)

	#y direction
	while(distance[4][6]>2):
		twist2.linear.y = 0.57*y_dir2
		find_positions()
		if (x_dir3 == -1):
			frcx = (x[4]*x_dir3 + desirex)/10
		else:
			frcx = (x[4]*x_dir3 - desirex)/10

		frcy = (y[4] - desirey)/10
		if (abs(frcx)>0.4):
			frcx = 0.4*x_dir3
		elif (abs(frcx)<0.05):
			frcx = 0
		if (abs(frcy)>0.4):
			frcy = 0.4*y_dir3
		elif (abs(frcy)<0.05):
			frcy = 0
		twist3.linear.x = frcx
		twist3.linear.y = frcy
		pub2.publish(twist2)
		pub3.publish(twist3)
		#print x[4],y[4]
	twist2.linear.x = 0
	twist2.linear.y = 0
	pub2.publish(twist2)

	return

#UAV3 Formation-------------------------------------
def UAV3_Formation():
	direc = dir_chk(1)
	x1 = direc[0]
	y1 = direc[1]
	temp1 = distance[3][6]
	temp2 = distance[3][5]

	chk = 0

	twist3.linear.x = 0
	twist3.linear.y = 0
	pub3.publish(twist3)
	return




#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
#PID Controller--------------------------------
pid_setx = 0
pid_sety = 0
def PID(data,desire):
	global I,prev_time,prev_err,pd_set
	error = (desire + data)/10
	if (abs(error)<0.05):
		error = 0
	#cur_time = time.time()
	#time_diff = cur_time - prev_time
	#err_diff = error - prev_err
	#P = error
	#I += error*time_diff
	#if (time_diff>0):
		#D = err_diff/time_diff
	#prev_err = error
	#prev_time = cur_time
	#temp = str(I)
	#print I
	return(error)

def PID_x(data,desire):
	global pid_setx
	error = (desire + data)/10
	if (abs(error)<0.05):
		error = 0
		pid_setx = 1

	return (error)

def PID_y(data,desire):
	global pid_sety
	error = (desire - data)/10
	if (abs(error)<0.05):
		error = 0
		pid_sety = 1

	return(error)

def stop():

	twist3.linear.y = 0
	twist2.linear.y = 0
	twist1.linear.y = 0

	pub3.publish(twist3)
	pub2.publish(twist2)
	pub1.publish(twist1)




rospy.init_node('uav_mover', anonymous=True)



pub3 = rospy.Publisher('uav3/cmd_vel', Twist, queue_size=1)

pub1 = rospy.Publisher('uav1/cmd_vel', Twist, queue_size=1)
pub2 = rospy.Publisher('uav2/cmd_vel', Twist, queue_size=1)






range_sensor3 = rospy.Publisher('uav3/range_sensor', Twist, queue_size=1)
range_sensor2 = rospy.Publisher('uav2/range_sensor', Twist, queue_size=1)
range_sensor1 = rospy.Publisher('uav1/range_sensor', Twist, queue_size=1)


twist1 = Twist()
twist2 = Twist()
twist3 = Twist()


range_sensor_data2 = Twist()
range_sensor_data1 = Twist()
range_sensor_data3 = Twist()




rospy.Subscriber('uav3/range_sensor', Twist, uav3_range, queue_size=1)
rospy.Subscriber('uav2/range_sensor', Twist, uav2_range, queue_size=1)
rospy.Subscriber('uav1/range_sensor', Twist, uav1_range, queue_size=1)




#-----------------------------Odometry-----------------------------------------

rospy.Subscriber('uav3/ground_truth/state', Odometry, callback3, queue_size=1)
rospy.Subscriber('uav2/ground_truth/state', Odometry, callback2, queue_size=1)
rospy.Subscriber('uav1/ground_truth/state', Odometry, callback1, queue_size=1)







formsn_set = [0,0,0,0,0]

reverse = 0
prev_rate = 0
sts = 0
go = 0

d = desirexy(2,2)
desirex = d[0]
desirey = d[1]
while(not rospy.on_shutdown(stop)):
	if (altitude[5]<2):
		twist3.linear.z = 0.5
		twist2.linear.z = 0.5
		twist1.linear.z = 0.5
		pub3.publish(twist3)
		pub2.publish(twist2)
		pub1.publish(twist1)
	else:

		if (sts==0):
			twist3.linear.z = 0
			twist2.linear.z = 0
			twist1.linear.z = 0
			pub3.publish(twist3)
			pub2.publish(twist2)
			pub1.publish(twist1)
			time.sleep(2)
			sts = 1
		else:
			if (formsn_set[4]==0):
				UAV2_Formation()
				formsn_set[4]=1
			else:
				find_positions()
				if (x_dir3 == -1):
					frcx = (x[4]*x_dir3 + desirex)/10
				else:
					frcx = (x[4]*x_dir3 - desirex)/10
				frcy = (y[4] - desirey)/10
				if (abs(frcx)>0.4):
					frcx = 0.4*x_dir3
				elif (abs(frcx)<0.05):
					frcx = 0
				if (abs(frcy)>0.4):
					frcy = 0.4*y_dir3
				elif (abs(frcy)<0.05):
					frcy = 0
				else:
					if (y_dir2==-1):
						frcy = (desirey - y[4])/10
				twist3.linear.x = frcx
				twist3.linear.y = frcy
				pub3.publish(twist3)



exit()
