#!/usr/bin/env python3

# Python module import
import numpy as np
import math as ma
import time


# Ppe module import
import frameHeader
import joint


########################################################
#                                                      #
#  !  This is not computing a Histogramm of Joints  !  #
#  !  This is only computing local 3D coordinates   !  #
#                                                      #
########################################################

# Die Basisfunktion des Modules
def compute_hoj3d( list_of_joints, reference_join, reference_join_up, reference_join_left, reference_join_right, joint_indexes = [], use_triangle_function = False, n_time = 0.0):

	t0 = time.time()

	# the historamm of joints 3D
	hoj3d = []

	# get joints to compute
	if(joint_indexes):
		joints_to_compute = []
		for index in joint_indexes:
			joints_to_compute.append(list_of_joints[index])
	else:
		joints_to_compute = list_of_joints

	#translation
	translation_vector = np.array([-reference_join.get_WorldJoint()[0], -reference_join.get_WorldJoint()[1], -reference_join.get_WorldJoint()[2]])

	# print(translation_vector)

	for joint in joints_to_compute:
		point = np.array(joint.get_WorldJoint())
		transformed_point = point + translation_vector
		joint.set_WorldJoint(transformed_point.item(0),transformed_point.item(1),transformed_point.item(2))
	
	#
	# X = right_hip to left_hip
	#
	# Y = bottom_center to spine
	# 		(allmost perpendicular to alpha)
	#
	x_vector = np.array(reference_join_left.get_WorldJoint()) - np.array(reference_join_right.get_WorldJoint())
	y_vector = np.array(reference_join_up.get_WorldJoint()) - np.array(reference_join.get_WorldJoint())

	# calculate z
	z_vector = np.cross(x_vector,y_vector)

	# recalculate y so it is perpendicular to XZ plane
	y_vector = np.cross(z_vector,x_vector)

	for joint in joints_to_compute:
		hoj3d.append(transform_coordinate(x_vector,y_vector,z_vector,joint.get_WorldJoint()))
		

	t1 = time.time()
	n_time += t1 - t0

	return hoj3d,n_time

def transform_coordinate(x,y,z,vector):
	vector = np.array(vector)
	x = np.array(x)
	y = np.array(y)
	z = np.array(z)

	x_len = vector_len(x)
	y_len = vector_len(y)
	z_len = vector_len(z)

	# old_vector = (x,y,z)
	# new_vector = (s,t,u)

	s_vector = ((vector * x) / (x_len * x_len)) * x
	t_vector = ((vector * y) / (y_len * y_len)) * y
	u_vector = ((vector * z) / (z_len * z_len)) * z
	
	s = vector_len(s_vector)
	t = vector_len(t_vector)
	u = vector_len(u_vector)
	
	
	# check if vector os in the same direction as the axis
	if (vector_len(s_vector + x) < vector_len(s_vector) or vector_len(s_vector + x) < vector_len(x)):
		s = -s;
	if (vector_len(t_vector + y) < vector_len(t_vector) or vector_len(t_vector + y) < vector_len(y)):
		t = -t;
	if (vector_len(u_vector + z) < vector_len(u_vector) or vector_len(u_vector + z) < vector_len(z)):
		u = -u;

	return s,t,u

def vector_len(vector):
	return (np.sqrt((vector * vector).sum()))