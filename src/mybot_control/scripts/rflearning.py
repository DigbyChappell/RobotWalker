#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty
import math
import numpy as np
from datetime import datetime
import time
from locator import coords, initialiseworld
from tf import transformations

alpha = 0.5;
gamma = 0.1;
eta = 0.1;


def train(pub1, pub2, pub3):
    rospy.wait_for_service('/gazebo/reset_simulation')
    reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    reset_simulation()
    time.sleep(0.5)
    rospy.wait_for_service('/gazebo/reset_world')
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    reset_world()
    time.sleep(0.5)
    # 6 possible actions - each joint to move by set amount
    Q = np.zeros((41, 11, 41, 11, 6))	# Foot1x, Foot1z, Foot2x, Foot2z, Action
    world = initialiseworld()
    for episode in range(1000):
	print('episode', episode)
	currAngles  = [0.8232, 1.4952, 0.8232]
	currState, reward = statefinder(currAngles, world)
	oldState = currState
	dth = 0.3

	for timestep in range(1000):
	    print('t = ', timestep)
	    oldState = currState
	    oldAngles = currAngles
	    a = actionselection(currState, Q, eta)
	    if a == 0:
		currAngles = np.add(oldAngles, [dth, 0, 0])
	    elif a == 1:
		currAngles = np.add(oldAngles, [-dth, 0, 0])
	    elif a == 2:
		currAngles = np.add(oldAngles, [0, dth, 0])
	    elif a == 3:
		currAngles = np.add(oldAngles, [0, -dth, 0])
	    elif a == 4:
		currAngles = np.add(oldAngles, [0, 0, dth])
	    elif a == 5:
		currAngles = np.add(oldAngles, [0, 0, -dth])
	    elif a == 6:
		break
	    try:
		motor_step(oldAngles, currAngles, 0.01, pub1, pub2, pub3)
		currState, reward = statefinder(currAngles, world)
		if reward > 1:
		    break
		print('joint angles:', currAngles)
		print('state:', currState)
		print('reward:', reward)
		if np.any(np.array(currState) < 0) or np.any(np.array(currState) > 40):
		    break
		[s0, s1, s2, s3] = oldState
		[sc0, sc1, sc2, sc3] = currState
		Qold = Q
	        Q[s0, s1, s2, s3, a] = (1-alpha)*Qold[s0, s1, s2, s3, a] + alpha*(reward + gamma*np.max(Q[sc0, sc1, sc2, sc3, :]))
	    except IndexError:
		break
	currAngles  = [0.8232, 1.4952, 0.8232]
	motor_step(oldAngles, currAngles, 0.01, pub1, pub2, pub3)
	rospy.wait_for_service('/gazebo/reset_simulation')
	reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
	reset_simulation()
	time.sleep(0.5)
	rospy.wait_for_service('/gazebo/reset_world')
	reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
	reset_world()
	time.sleep(0.5)
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
    speed = 0.2*160/180
    dt = dt/3
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


def statefinder(currAngles, world):
    th1, th2, th3 = currAngles
    positions = [[np.linspace(1.0,-1.0,41)], 
	         [np.linspace(0,1.0,11)],
	         [np.linspace(1.0,-1.0,41)], 
	         [np.linspace(0,1.0,11)]] 
    
    foot1coords = coords('mybot', 'LOIII')

    foot1xpos = foot1coords[0]
    foot1zpos = foot1coords[2]

    R = transformations.quaternion_matrix(foot1coords[3:])
    foot2x_rframe = 0.15*np.sin(th1) + 0.15*np.sin(th1+th2) + 0.131*np.sin(th1+th2+th3)
    foot2y_rframe = 0
    foot2z_rframe = 0.141 + 0.15*np.cos(th1) + 0.15*np.cos(th1+th2) + 0.131*np.cos(th1+th2+th3)
    foot2_wframe = np.matmul(R,[foot2x_rframe, foot2y_rframe, foot2z_rframe, 1])

    foot2xpos = foot1xpos + foot2_wframe[0]
    foot2zpos = foot1zpos + foot2_wframe[2]

    foot1x = int(np.round((foot1xpos-1.0)/(-0.05)))
    foot1z = np.max((int(np.round(foot1zpos/(0.1))),0))
    foot2x = int(np.round((foot2xpos-1.0)/(-0.05)))
    foot2z = np.max((int(np.round(foot2zpos/(0.1))),0))

    # state and reward
    state = [foot1x, foot1z, foot2x, foot2z]
    goalx = coords('goal', '')[0]
    reward = world[state[0], state[1]] + (1.0 - abs(np.min(((foot1xpos - goalx), (foot2xpos - goalx)))))*0.5

    # if it's fallen over but hasn't reached the goal, penalise
    if foot1zpos > 0.01 and foot2zpos > 0.01 and reward < 1:
	reward = -1
    # if it's reached joint limits, penalise
    if th1 > 2.17 or th1 < -1.17:
	reward = -1
    if th2 > 2.00 or th2 < -0.86:
	reward = -1
    if th3 > 1.98 or th3 < -0.77:
	reward = -1

    return state, reward


def actionselection(currState, Q, eta):
    action_rewards = Q[currState[0], currState[1], currState[2], currState[3], :]
    max_reward = np.max(action_rewards)
    if np.any(abs(action_rewards-max_reward)<eta):
	possible_actions = np.transpose(np.argwhere(action_rewards-max_reward<eta))[0]
	action = np.random.choice(possible_actions, 1)
    else:    
	action = np.argmax(action_rewards)
    if np.all(action_rewards<0):
	action = 6
    print('action rewards:', action_rewards)
    print('chosen action:', action)
    return action

