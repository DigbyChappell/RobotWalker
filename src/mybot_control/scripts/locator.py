#! /usr/bin/env python

from gazebo_msgs.srv import GetModelState, GetWorldProperties, GetModelProperties
import rospy
import numpy as np
import tf

class Block:
    def __init__(self, name, relative_entity_name):
        self._name = name
        self._relative_entity_name = relative_entity_name


def coords(blockname, relativeblockname):

    block = Block(blockname, relativeblockname)

    try:
        rospy.wait_for_service('/gazebo/get_model_state', timeout=2)
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates(block._name, '')
	x = resp_coordinates.pose.position.x
	y = resp_coordinates.pose.position.y
	z = resp_coordinates.pose.position.z
        qi = resp_coordinates.pose.orientation.x
	qj = resp_coordinates.pose.orientation.y
	qk = resp_coordinates.pose.orientation.z
	qr = resp_coordinates.pose.orientation.w
    except rospy.ServiceException as e:
        [x, y, z, qi, qj, qk, qr] = [0, 0, 0, 0, 0, 0, 0]
    return np.array([x, y, z, qi, qj, qk, qr])


def initialiseworld(xs,zs):
    rospy.wait_for_service('/gazebo/get_world_properties', timeout=2)
    world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    names = world_properties().model_names

    world = np.zeros((xs, zs))
    for model in names:
	if model == 'goal':
	    goal = coords(model, '')
	    goalx = int(np.round((goal[0]-1.0)/(2/(1-xs))))
	    goalz = int(np.round(goal[2]/(1/(zs-1.0))))
	    world[goalx, goalz] = 1
	elif model != 'mybot' and model != 'ground_plane':
	    rospy.wait_for_service('/gazebo/get_model_properties', timeout=2)
	    obst_properties = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
	    obst_props = obst_properties(model)
	    bodyname = obst_props.body_names[0]
	    dimensions = bodyname.split('x')
	    xsize = float(dimensions[0])
	    zsize = float(dimensions[2])
	    obst = coords(model, '')
	    # bounding box around obstacle
	    obstx1 = int(np.round((obst[0]+xsize/2-1.0)/(2/(1.0-xs))))
	    obstz1 = int(np.round((obst[2]-zsize/2)/(1/(zs-1.0))))
	    obstx2 = int(np.round((obst[0]-xsize/2-1.0)/(2/(1.0-xs))))
	    obstz2 = int(np.round((obst[2]+zsize/2)/(1/(zs-1.0))))
	    world[obstx1:obstx2+1, obstz1:obstz2+1] = -1
    return world


