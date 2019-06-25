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
gamma = 0.2
eta = 0.1

xs = 41; xmin = 1.0; xmax = -1.0
zs = 41; zmin = 0.0; zmax = 1.0
actions = 3

dt=0.1

def train3(pub1, pub2, pub3):
    Q = np.zeros((xs, zs, xs, zs, actions))
    # reset simulation
    oldAngles  = [0.8232, 1.4952, 0.8232]
    currAngles  = [0.8232, 1.4952, 0.8232]
    motor_step(oldAngles, currAngles, dt, pub1, pub2, pub3)
    time.sleep(2.0)
    sumrewards = []
    for episode in range(1000):
	print'episode:	', episode
	# reset simulation
	rewardbuffer = np.zeros((5, 1))
	currAngles  = [0.8232, 1.4952, 0.8232]
	motor_step(oldAngles, currAngles, dt, pub1, pub2, pub3)
	time.sleep(2.0)
	resetsim()

	currState = statefinder(currAngles)
	oldState = currState
	sumreward = 0
	goaldist = 1.0
	for timestep in range(100):
	    print't = 	', timestep
	    oldState = currState
	    oldAngles = currAngles
	    a = actionselection(currState, Q, eta)
	    try:
		    if a == 0:
			f1state, f2state = statefinder(oldAngles)
			f1old = [xmin+f1state[0]*(xmax-xmin)/xs, zmin + f1state[1]*(zmax-zmin)/zs]
			f2old = [xmin+f2state[0]*(xmax-xmin)/xs, zmin + f2state[1]*(zmax-zmin)/zs]
			nextf1state = np.add(f1state, [1,0])
			nextf2state = np.add(f2state, [1,0])
			f1new = [xmin+nextf1state[0]*(xmax-xmin)/xs, zmin + nextf1state[1]*(zmax-zmin)/zs]
			f2new = [xmin+nextf2state[0]*(xmax-xmin)/xs, zmin + nextf2state[1]*(zmax-zmin)/zs]
			currAngles = steptoposition(f1old,f2old,f1new,f2new,pub1,pub2,pub3)
		    elif a == 1:
			f1state, f2state = statefinder(oldAngles)
			f1old = [xmin+f1state[0]*(xmax-xmin)/xs, zmin + f1state[1]*(zmax-zmin)/zs]
			f2old = [xmin+f2state[0]*(xmax-xmin)/xs, zmin + f2state[1]*(zmax-zmin)/zs]
			nextf1state = np.add(f1state, [1,1])
			nextf2state = np.add(f2state, [1,1])
			f1new = [xmin+nextf1state[0]*(xmax-xmin)/xs, zmin + nextf1state[1]*(zmax-zmin)/zs]
			f2new = [xmin+nextf2state[0]*(xmax-xmin)/xs, zmin + nextf2state[1]*(zmax-zmin)/zs]
			currAngles = steptoposition(f1old,f2old,f1new,f2new,pub1,pub2,pub3)
		    elif a == 2:
			currAngles = yeetit(currAngles, pub1, pub2, pub3)
	    except ValueError:
		print 'failed step'
	    motor_step(oldAngles, currAngles, dt, pub1, pub2, pub3)
	    currState = statefinder(currAngles)
	    reward, goaldist = rewardfunc(currAngles, goaldist, timestep)
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
	    try:
	        Q = updateQ(Q, currState, oldState, reward, a, alpha, gamma)
	    except IndexError:
		print 'failed step'
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


def anglesfromposition(f1,f2):
    x = f2[0] - f1[0]
    z = (f1[1]+0.141)-(f2[1]+0.131)
    l = np.sqrt([np.power(x,2) + np.power(z,2)])
    beta = np.arccos(1.0-np.power(l,2)/(2.0*np.power(0.15,2)))
    alpha = (np.pi-beta)/2
    gamma = np.arctan(z/x)

    th1 = np.pi-(np.pi/2-gamma)-alpha
    th2 = np.pi-beta
    th3 = np.pi-th1-th2
    return [th1, th2, th3]


def statefinder(currAngles):
    th1, th2, th3 = currAngles
    positions = [[np.linspace(1.0,-1.0,41)], 
	         [np.linspace(0,1.0,11)],
	         [np.linspace(1.0,-1.0,41)], 
	         [np.linspace(0,1.0,11)]] 
    
    foot1coords = coords('mybot', 'LOIII')
    R = transformations.quaternion_matrix(foot1coords[3:])

    foot2basex_rframe = 0.15*np.sin(th1) + 0.15*np.sin(th1+th2) + 0.131*np.sin(th1+th2+th3)
    foot2basey_rframe = 0
    foot2basez_rframe = 0.141 + 0.15*np.cos(th1) + 0.15*np.cos(th1+th2) + 0.131*np.cos(th1+th2+th3)
    foot2base_wframe = np.matmul(R,[foot2basex_rframe, foot2basey_rframe, foot2basez_rframe, 1])

    foot1x = foot1coords[0]
    foot1z = foot1coords[2]
    foot2x = foot1coords[0] + foot2base_wframe[0]
    foot2z = foot1coords[2] + foot2base_wframe[2]
    
    # quantise values to states
    foot1xstate = np.max((int(np.round(xs*(foot1x-xmin)/(xmax-xmin))), 0))
    foot1zstate = np.max((int(np.round(zs*(foot1z-zmin)/(zmax-zmin))), 0))
    foot2xstate = np.max((int(np.round(xs*(foot2x-xmin)/(xmax-xmin))), 0))
    foot2zstate = np.max((int(np.round(zs*(foot2z-zmin)/(zmax-zmin))), 0))
    
    f1state = [foot1xstate, foot1zstate]
    f2state = [foot2xstate, foot2zstate]

    # state
    state = [f1state, f2state]

    return state


def actionselection(currState, Q, eta):
    [x1, z1], [x2, z2] = currState
    try:
        action_rewards = Q[x1, z1, x2, z2, :]
	possible_rewards = action_rewards
	max_reward = np.max(possible_rewards)
	possible_actions = np.transpose(np.argwhere(abs(possible_rewards - max_reward)<eta))[0]
	# if any are close to maximum reward, choose randomly
	action = np.int(np.random.choice(possible_actions, 1))
	print 'possible actions:', possible_actions[:]
	print 'action rewards:', possible_rewards[:]
	print 'chosen action:', action
    except IndexError:
	action = 0
    return action


def rewardfunc(angles, prevdist, time):
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
    goaldist = np.min((foot1base-goal, foot1tip-goal, foot2base-goal, foot2tip-goal))
    distreward = prevdist - goaldist
    print '-------------------------distance reward	:', distreward
    # time factor - decrease reward for slower policies
    timefactor = 1.0 - np.exp(-time/100.0)
    print '-------------------------time penalty		:', -timefactor

    # tilt penalty - don't want the robot to reward falling over sideways
    tiltpenalty = -abs(yaw/(np.pi))
    print '-------------------------tilt penalty		:', tiltpenalty
    # reward
    reward = distreward + tiltpenalty - timefactor
    return reward, goaldist


def steptoposition(f1old, f2old, f1new, f2new, pub1, pub2, pub3):
    startAngles = anglesfromposition(f1old, f2old)
    # raise leg1
    iAngles = anglesfromposition(np.add(f1old, [0, 0.02]), f2old)
    # extend across
    iiAngles = anglesfromposition(np.add(f1new, [0, 0.02]), f2old)
    # lower leg1
    iiiAngles = anglesfromposition(f1new, f2old)
    # raise leg2
    ivAngles = anglesfromposition(f1new, np.add(f2old, [0,0.02]))
    # close together
    vAngles = anglesfromposition(f1new, np.add(f2new, [0,0.02]))
    # lower leg2
    viAngles = anglesfromposition(f1new, f2new)
    motor_step(startAngles, iAngles, dt, pub1, pub2, pub3)
    motor_step(iAngles, iiAngles, dt, pub1, pub2, pub3)
    motor_step(iiAngles, iiiAngles, dt, pub1, pub2, pub3)
    motor_step(iiiAngles, ivAngles, dt, pub1, pub2, pub3)
    motor_step(ivAngles, vAngles, dt, pub1, pub2, pub3)
    motor_step(vAngles, viAngles, dt, pub1, pub2, pub3)

    finalAngles = viAngles
    return finalAngles
    

def yeetit(angles, pub1, pub2, pub3):
    yeetangles = [[ 0.8232,  1.4952,  0.8232],
		  [-0.2777,  2.1059,  0.5280],
		  [-0.0141,  0.7994,  1.5709],
		  [-0.1651,  0.3641,  1.3718],
		  [-1.1700, -0.4284, -0.8458]]
    dts = [0.1, 0.1, 0.001, 0.1]
    for i in range(len(yeetangles)-1):
	dt = dts[i]
	th_old = yeetangles[i]
	th_new = yeetangles[i+1]
	motor_step(th_old, th_new, dt, pub1, pub2, pub3)
    return th_new


def updateQ(Q, oldState, currState, reward, a, alpha, gamma):
    [s0, s1], [s2, s3] = oldState
    [sc0, sc1], [sc2, sc3] = currState    
    Qold = Q
    Q[s0, s1, s2, s3, a] = (1-alpha)*Qold[s0, s1, s2, s3, a] + alpha*(reward + gamma*np.max(Q[sc0, sc1, sc2, sc3, :]))
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
