#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import math
import numpy as np
from datetime import datetime
import time

 
def walker(pub1, pub2, pub3):
    
    stepangles = [[0.8232, 1.4952, 0.8232], 
		  [1.1862, 1.3853, 0.5701],
		  [1.7010, 0.2112, 1.2283],
		  [1.3119, 0.5178, 1.3119],
		  [1.2283, 0.2112, 1.7010],
		  [0.5701, 1.3853, 1.1862],
		  [0.8232, 1.4952, 0.8232]]
    yeetangles = [[ 0.8232,  1.4952,  0.8232],
		  [-0.2777,  2.1059,  0.5280],
		  [-0.0141,  0.7994,  1.5709],
		  [-0.1651,  0.3641,  1.3718],
		  [-1.1700, -0.4284, -0.8458]]
    th_old = stepangles[0]
    th_new = stepangles[0]
    time.sleep(2)
    motor_step(th_old, th_new, 0.1, pub1, pub2, pub3)    
    starttime = datetime.now()
    for j in range(4):
	for i in range(len(stepangles)-1):
		dt = 0.1
		th_old = stepangles[i]
		th_new = stepangles[i+1]
		runtime = datetime.now() - starttime
		message = str(runtime) + ',' + str(th_old[0]) + ',' + str(th_old[1]) + ',' + str(th_old[2])
		rospy.loginfo(message)
		motor_step(th_old, th_new, dt, pub1, pub2, pub3)
		time.sleep(0.25)

    dts = [0.1, 0.1, 0.001, 0.1]
    for i in range(len(yeetangles)-1):
	dt = dts[i]
	th_old = yeetangles[i]
	th_new = yeetangles[i+1]
	runtime = datetime.now() - starttime
	message = str(runtime) + ',' + str(th_old[0]) + ',' + str(th_old[1]) + ',' + str(th_old[2])
	rospy.loginfo(message)
	motor_step(th_old, th_new, dt, pub1, pub2, pub3)
    return


def motor_convert(th):
    th_m = np.zeros((3,1))
    th0_0 = 0.65
    th0_90 = 0.18
    th_m[0] = th0_0 + (th0_90-th0_0)*th[0]/(math.pi/2)
    th1_0 = 0.15
    th1_90 = 0.70
    th_m[1] = th1_0 + (th1_90-th1_0)*th[1]/(math.pi/2)
    th2_0 = 0.72
    th2_90 = 0.20
    th_m[2] = th2_0 + (th2_90-th2_0)*th[2]/(math.pi/2)
    return th_m


def motor_step(th_old, th_new, dt, pub1, pub2, pub3):
    speed = 2*160/180 # don't know why wtf
    th_old_m = motor_convert(th_old)
    th_new_m = motor_convert(th_new)
    motion_time = np.max(abs(th_new_m-th_old_m))/speed
    N_steps = np.ceil(motion_time/dt)
    th_t = np.zeros((np.int(N_steps), 3))
    for i in range(len(th_new)):
	th_t[:, i] = np.linspace(th_old[i], th_new[i], N_steps)
    for i in range(np.int(N_steps)):
	pub1.publish(th_t[i, 0])
	pub2.publish(th_t[i, 1])
	pub3.publish(th_t[i, 2])
	time.sleep(dt)
    return
