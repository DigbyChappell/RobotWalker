#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty
import math
import numpy as np
from scipy import ndimage
from datetime import datetime
import time
from locator import coords
from tf import transformations
import matplotlib.pyplot as plt

alpha = 0.5
gamma = 0.1
eta = 0.0001

xs = 41; xmin = 1.0; xmax = -1.0
zs = 11; zmin = 0.0; zmax = 1.0
th1s = 11; th1min = -1.17; th1max = 2.15; dth1 = (th1max-th1min)/th1s
th2s = 11; th2min = -0.86; th2max = 2.00; dth2 = (th2max-th2min)/th2s
th3s = 11; th3min = -0.77; th3max = 1.98; dth3 = (th3max-th3min)/th3s
actions = 6

dt=0.1

def train2(pub1, pub2, pub3):
    Q = np.zeros((xs, zs, th1s, th2s, th3s, actions))
    # reset simulation
    oldAngles  = [0.8232, 1.4952, 0.8232]
    currAngles  = [0.8232, 1.4952, 0.8232]
    motor_step(oldAngles, currAngles, dt, pub1, pub2, pub3)
    time.sleep(4.0)
    # initialise Q with human demonstrations
    for i in range(5):
	resetsim()
	time.sleep(4.0)
	Q = walker2(pub1, pub2, pub3, Q)
	oldAngles  = [0.8232, 1.4952, 0.8232]
    for i in range(th1s):
	for j in range(th2s):
	    for k in range(th3s):
		for a in range(actions):
		    Q[:,:,i,j,k,a] = ndimage.morphology.white_tophat(Q[:,:,i,j,k,a],size=5)
    for episode in range(1000):
	print'episode:	', episode
	# reset simulation
	rewardbuffer = np.zeros((20, 1))
	currAngles  = [0.8232, 1.4952, 0.8232]
	motor_step(oldAngles, currAngles, dt, pub1, pub2, pub3)
	time.sleep(4.0)
	resetsim()

	currState = statefinder(currAngles)
	oldState = currState

	for timestep in range(100):
	    print't = 	', timestep
	    oldState = currState
	    oldAngles = currAngles
	    a = actionselection(currState, Q, eta)
	    if a == 0:
		currAngles = np.add(oldAngles, [dth1, 0, 0])
	    elif a == 1:
		currAngles = np.add(oldAngles, [-dth1, 0, 0])
	    elif a == 2:
		currAngles = np.add(oldAngles, [0, dth2, 0])
	    elif a == 3:
		currAngles = np.add(oldAngles, [0, -dth2, 0])
	    elif a == 4:
		currAngles = np.add(oldAngles, [0, 0, dth3])
	    elif a == 5:
		currAngles = np.add(oldAngles, [0, 0, -dth3])
	    elif a == 6:
		break
	    motor_step(oldAngles, currAngles, dt, pub1, pub2, pub3)
	    currState = statefinder(currAngles)
	    reward = rewardfunc(currAngles, timestep)
	    rewardbuffer = np.roll(rewardbuffer, -1)
	    rewardbuffer[len(rewardbuffer)-1] = reward
	    print'joint angles:	', currAngles
	    print'state:	', currState
	    print'reward:	', reward
	    print'reward buffer:	'
	    print rewardbuffer
	    print''
	    if np.any(np.array(currState) < 0) or np.any(np.array(currState) > 40):
	        break
	    if reward <= -1.0 or reward >= 1.0:
		break
	    if np.all(rewardbuffer<0):
		break
	    Q = updateQ(Q, currState, oldState, reward, a, alpha, gamma)
	    sumreward += reward
	print''
	print'---------------------------'
	sumrewards.append(sumreward)
    plt.plot(sumrewards,range(1000))
    plt.show()
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
    dt = dt/2
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


def walker2(pub1, pub2, pub3, Q):
    oldAngles = [0.8232, 1.4952, 0.8232]
    timestep = 0
    for j in range(5):
	dt=0.5 
	oldAngles = [0.8232, 1.4952, 0.8232] + np.random.normal(0,0.05,(1,3))[0]
	oldState = statefinder(oldAngles)
	actions = [5,0,3,3,4,0,3,3,3,0,4,4,1,4,1,5,1,2,2,2,5,1,2,2,5,0]
	for a in actions:
	    print'action', a
	    if a == 0:
		currAngles = np.add(oldAngles, [dth1, 0, 0])
	    elif a == 1:
		currAngles = np.add(oldAngles, [-dth1, 0, 0])
	    elif a == 2:
		currAngles = np.add(oldAngles, [0, dth2, 0])
	    elif a == 3:
		currAngles = np.add(oldAngles, [0, -dth2, 0])
	    elif a == 4:
		currAngles = np.add(oldAngles, [0, 0, dth3])
	    elif a == 5:
		currAngles = np.add(oldAngles, [0, 0, -dth3])
	    elif a == 6:
		break
	    motor_step(oldAngles, currAngles, dt, pub1, pub2, pub3)
	    currState = statefinder(currAngles)
	    reward = rewardfunc(currAngles, timestep)
	    print'joint angles:	', currAngles
	    print'state:	', currState
	    print'reward:	', reward
	    print''
	    Q = updateQ(Q, currState, oldState, reward, a, alpha, gamma)
	    timestep += 1
	    oldAngles = currAngles
	    oldState = currState

    oldAngles = [0.8232, 1.4952, 0.8232]
    oldState = statefinder(oldAngles)
    actions = [1,1,2,1,1,4,3,4,3,3,3,3,5,5,5,3,1,5,3,1,5,3,5]
    for a in actions:
	print 'action', a
	if a == 0:
	    currAngles = np.add(oldAngles, [dth1, 0, 0])
	elif a == 1:
	    currAngles = np.add(oldAngles, [-dth1, 0, 0])
	elif a == 2:
	    currAngles = np.add(oldAngles, [0, dth2, 0])
	elif a == 3:
	    currAngles = np.add(oldAngles, [0, -dth2, 0])
	elif a == 4:
	    currAngles = np.add(oldAngles, [0, 0, dth3])
	elif a == 5:
	    currAngles = np.add(oldAngles, [0, 0, -dth3])
	elif a == 6:
	    break
	motor_step(oldAngles, currAngles, dt, pub1, pub2, pub3)
	currState = statefinder(currAngles)
	reward = rewardfunc(currAngles, timestep)
	print'joint angles:	', currAngles
	print'state:	', currState
	print'reward:	', reward
	print''
	Q = updateQ(Q, currState, oldState, reward, a, alpha, gamma)
	timestep += 1
	oldAngles = currAngles
	oldState = currState
    currAngles  = [0.8232, 1.4952, 0.8232]
    motor_step(oldAngles, currAngles, dt, pub1, pub2, pub3)
    return Q


def statefinder(currAngles):
    th1, th2, th3 = currAngles
    positions = [[np.linspace(1.0,-1.0,41)], 
	         [np.linspace(0,1.0,11)],
	         [np.linspace(1.0,-1.0,41)], 
	         [np.linspace(0,1.0,11)]] 
    
    foot1coords = coords('mybot', 'LOIII')

    foot1x = foot1coords[0]
    foot1z = foot1coords[2]
    
    # quantise values to states
    foot1xq = np.max((int(np.round(xs*(foot1x-xmin)/(xmax-xmin))), 0))
    foot1zq = np.max((int(np.round(zs*(foot1z-zmin)/(zmax-zmin))), 0))
    th1q = np.max((int(np.round(th1s*(th1-th1min)/(th1max-th1min))), 0))
    th2q = np.max((int(np.round(th2s*(th2-th2min)/(th2max-th2min))), 0))
    th3q = np.max((int(np.round(th3s*(th3-th3min)/(th3max-th3min))), 0))

    # state
    state = [foot1xq, foot1zq, th1q, th2q, th3q]

    return state


def actionselection(currState, Q, eta):
    x, z, th1, th2, th3 = currState
    try:
        action_rewards = Q[x, z, th1, th2, th3, :]
	banned_actions = actionchecker(currState)
	possible_rewards = banned_actions+action_rewards
	max_reward = np.max(possible_rewards)
	possible_actions = np.transpose(np.argwhere(abs(possible_rewards - max_reward)<eta))[0]
	# if any are close to maximum reward, choose randomly
	action = np.int(np.random.choice(possible_actions, 1))
	print 'possible actions:', possible_actions[:]
	print 'action rewards:', possible_rewards[:]
	print 'chosen action:', action
    except IndexError:
	action = 6
    return action


def actionchecker(currState):
    x, z, th1, th2, th3 = currState
    banned_actions = np.zeros((1, actions))[0]
    if (th1 + 1 > th1s-1):
	banned_actions[0] = -99
    if (th1 - 1 < 0):
	banned_actions[1] = -99
    if (th2 + 1 > th2s-1):
	banned_actions[2] = -99
    if (th2 - 1 < 0):
	banned_actions[3] = -99
    if (th3 + 1 > th3s-1):
	banned_actions[4] = -99
    if (th3 - 1 < 0):
	banned_actions[5] = -99
    print banned_actions
    return banned_actions


def rewardfunc(angles, time):
    th1, th2, th3 = angles
    foot1coords = coords('mybot', 'LOIII')
    goalcoords = coords('goal', '')

    R = transformations.quaternion_matrix(foot1coords[3:])
    roll, pitch, yaw = transformations.euler_from_quaternion(foot1coords[3:])

    foot1tip_rframe = [0.2, 0, 0, 1]
    foot1tip_wframe = np.matmul(R, foot1tip_rframe)

    foot2basex_rframe = 0.15*np.sin(th1) + 0.15*np.sin(th1+th2) + 0.131*np.sin(th1+th2+th3)
    foot2basey_rframe = 0
    foot2basez_rframe = 0.141 + 0.15*np.cos(th1) + 0.15*np.cos(th1+th2) + 0.131*np.cos(th1+th2+th3)
    foot2base_wframe = np.matmul(R,[foot2basex_rframe, foot2basey_rframe, foot2basez_rframe, 1])

    foot2tipx_rframe = foot2basex_rframe - 0.2*np.cos(th1+th2+th3)
    foot2tipy_rframe = 0
    foot2tipz_rframe = foot2basez_rframe - 0.2*np.sin(th1+th2+th3)
    foot2tip_wframe = np.matmul(R, [foot2tipx_rframe, foot2tipy_rframe, foot2tipz_rframe, 1])

    # get x position of each foot's base and tip, and goal
    foot1base = foot1coords[0]
    foot1tip = foot1coords[0] + foot1tip_wframe[0]
    foot2base = foot1coords[0] + foot2base_wframe[0]
    foot2tip = foot1coords[0] + foot2tip_wframe[0]
    goal = goalcoords[0]

    # get minimum distance to goal
    distreward = 1.05 - np.min((foot1base-goal, foot1tip-goal, foot2base-goal, foot2tip-goal))
    print '-------------------------distance reward	:', distreward
    # time factor - decrease reward for slower policies
    timefactor = np.exp(-time/0.01)

    # tilt penalty - don't want the robot to reward falling over sideways
    tiltpenalty = -abs(yaw/(np.pi))
    print '-------------------------tilt penalty		:', tiltpenalty
    # reward
    reward = (distreward + tiltpenalty)*1.0
    return reward


def updateQ(Q, oldState, currState, reward, a, alpha, gamma):
    [s0, s1, s2, s3, s4] = oldState
    [sc0, sc1, sc2, sc3, sc4] = currState    
    Qold = Q
    Q[s0, s1, s2, s3, s4, a] = (1-alpha)*Qold[s0, s1, s2, s3, s4, a] + alpha*(reward + gamma*np.max(Q[sc0, sc1, sc2, sc3, sc4, :]))
    return Q


def resetsim():
    rospy.wait_for_service('/gazebo/reset_simulation')
    reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    reset_simulation()
    time.sleep(0.5)
    rospy.wait_for_service('/gazebo/reset_world')
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    reset_world()
    time.sleep(0.5)
    return
