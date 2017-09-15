#!/usr/bin/env/python3

# Python module import

import sys
import os
import math as ma
import argparse
import time as tM
import datetime as dT
import shutil 

# Ppe module import

import load_skeleton as l_S
import hoj3d as h3d
import hoj3d_tester as h3d_t

# ----------------------------------------------------------------------------------------------------------------------------------------------------------

def main():
	path_name = ""
	som_path = ""
	action_list = None
	ignore_tail = False
	verbose = False

	index_exceptions_fH = open('index_errors', 'a')
	inconsistency_exception_fH = open('some_more_inconsistent_file', 'a')
	nan_exception_fH = open('nan_errors', 'a')


	# Statisitcal values 
	number_of_consistent_data_sets = 0

	number_of_inconsistent_data_sets = 0

	number_of_nan_in_datasets = 0

	number_of_datasets_with_occlusion = 0

	# Parse the command line options.
	path_name, skeleton_name, action_list, ignore_tail, verbose = parseOpts( sys.argv )

	# ----------------------------------------------------------------------------------------------------
	# Read configuration files 

	# Read list of missed or incomplete skelelton files.
	_missed_skeletons_fileHandler = open('conf/missed_skeletons','r')
	_list_of_missed_skeletons_ = _missed_skeletons_fileHandler.readlines()

	# ----------------------------------------------------------------------------------------------------

	# The loop index for the list 
	i = 0

	# List all skeleton files in the data directory.
	dir_list = os.listdir(path_name)
	#dir_list = os.listdir("../skeleton")
	dir_list = sorted(dir_list)

	if( skeleton_name is not "*"):
		# Crop the list up to the point of the index of the given skeleton_name 
		# Then the list starts at this point
		dir_list = dir_list[dir_list.index(skeleton_name): ]

	# Variables for statistics
	computational_start_time = tM.time()

	# Step through the listed files in the dir_list
	for file in dir_list[i:]:

		# Check if the choosen file is in the list of files with missed or incomplete data.
		if( check_for_file_consistence( file.split(".")[0], _list_of_missed_skeletons_ ) is False ):

			# Build the whole filename with absolute path
			_skeleton_filename_ = path_name + file

			# Check for an action we want ( stored in the action list )
			if( is_action( action_list, _skeleton_filename_ ) ):

				# ----------------------------------------------------------------------------------------------------
				# Open the skeleton file if it exist.
				if( os.path.isfile( _skeleton_filename_ ) == True ):
					print( "Skeleton file: ", _skeleton_filename_ )
					_skeleton_fileHandler_ = open( _skeleton_filename_ , 'r')

					# Read the data from the skeleton file for the whole sequence
					all_skeleton_frames = l_S.read_skeleton_data( _skeleton_fileHandler_, ignore_tail, verbose )

				else:
					print("\nNo skeleton file with name: ", _skeleton_filename_ )
					print("Leave script now.\n")
					exit(0)

				# ----------------------------------------------------------------------------------------------------
				# Moin Franz,
				# Ich hab dir hier den Funktionsaufruf für die Hoj3D Funktion schon definiert.
				# Du brauchst dafür nur die Skeleton daten als Input. ( Soweit ich mich erinnere )
				# Aufbau der Daten:
				#
				#	 all_skeleton_frames ( liste von frames )
				#					|_> jeder Frame enthält den Frame_header und eine Liste von Joints des zugehörigen Skeletons 
				#
				#	-> frameHeader.py und joint.py sollten dir weitere Informationen dazu liefern
				#	-> Sollten weitere Fragen auftauchen -> schreib mir ne Mail, ich versuch sie trotz Urlaub so schnell wie möglich zu 
				#	   beantworten
				#
				#	# Franz 

				if os.path.exists(os.path.splitext(file)[0]):
					os.makedirs(os.path.splitext(file)[0])

				i = 0
				for frame in all_skeleton_frames:

					try:

						list_of_joints = frame.get_ListOfJoints()

						# gget joints from the paper 3, 5, 9, 6, 10, 13, 17, 14, 18, 12, 16
						# joints_to_compute = []
						# joints_to_compute.append(list_of_joints[3])		# head 		0
						# joints_to_compute.append(list_of_joints[5])		# l elbow	1
						# joints_to_compute.append(list_of_joints[9])		# r elbow	2
						# joints_to_compute.append(list_of_joints[6])		# l hand 	3
						# joints_to_compute.append(list_of_joints[10])		# r hand 	4
						# joints_to_compute.append(list_of_joints[13])		# l knee 	5
						# joints_to_compute.append(list_of_joints[17])		# r knee 	6
						# joints_to_compute.append(list_of_joints[14])		# l feet 	7
						# joints_to_compute.append(list_of_joints[18])		# r feet 	8
						
						hoj3d_set,time = h3d.compute_hoj3d(list_of_joints, list_of_joints[0], list_of_joints[1], list_of_joints[16], list_of_joints[12], joint_indexes=[3, 5, 9, 6, 10, 13, 17, 14, 18], use_triangle_function=True) # hip center, spine, hip right, hip left

						# If a NaN warning is raised in the h3d.compute function the process will stop for this set and all the previously computed frames of this set will be removed
						if( time is -1 ):
							number_of_nan_in_datasets += 1
							print("\n")			
							print("!!! Actual set " + _skeleton_filename_ + " has at least one NaN value error. !!!\n\n")
							name = _skeleton_filename_.split(".")[0].split("/")[7]
							nan_exception_fH.write( name+"\n" )
							del_path = '../hoj_test/'+name
							shutil.rmtree(del_path)
							break
						else:
							# testing
							test_filename = os.path.splitext(file)[0] + "/" + os.path.splitext(file)[0] + "_{0:0=3d}".format(i)
							h3d_t.write_hoj3d(test_filename,hoj3d_set)
							i += 1
							#Store statistic data
							number_of_consistent_data_sets += 1
		
					except IndexError:
						number_of_datasets_with_occlusion += 1
						print("\n")
						print("!!! Actual set " + _skeleton_filename_ +" has index error(s). !!! \n\n")
						index_exceptions_fH.write(os.path.basename(_skeleton_filename_)+"\n" )
		else:
			#Store statistic data
			number_of_inconsistent_data_sets += 1
			print("\n")			
			print("!!! Actual set " + _skeleton_filename_ + " has missing or incomplete skeleton data. !!!\n\n")
			inconsistency_exception_fH.write( os.path.basename(_skeleton_filename_)+"\n" )

	computational_end_time = tM.time()
	timeDiff = dT.timedelta(seconds=computational_end_time - computational_start_time)

	index_exceptions_fH.close()
	inconsistency_exception_fH.close()
	nan_exception_fH.close()

	print("\n\nSome statistics: ---------------------------------------------------------------------------------------------------------------")
	print("\n")
	print("Number of correct computed datasets        : ", number_of_consistent_data_sets)
	print("Number of inconsistent datasets            : ", number_of_inconsistent_data_sets)
	print("Number of datasets with NaN entrys         : ", number_of_nan_in_datasets)
	print("Number of datasets wth occluded body parts : ", number_of_datasets_with_occlusion)
	print("----------------------------------------------------------")
	print("Used time for the computation              : ", timeDiff )

# ----------------------------------------------------------------------------------------------------------------------------------------------------------

# A small function to skip actions which are not in the action list
def is_action( _action_list_, _skeleton_filename_ ):
	# If an action_list is given 
	if( _action_list_ is not None ):
		for key in _action_list_:
			if( key in _skeleton_filename_ ):
				# If the action of the skeleton file is in the action list.
				return True
	# If no action list is given
	else:
		return True

	# If the action of the skeleton file is not in the action list.
	return False		

# ----------------------------------------------------------------------------------------------------------------------------------------------------------

# A small function to check wether a file with incomplete data was choosen or not.
def check_for_file_consistence( _skeleton_filename_, _list_of_missed_skeletons_ ):
	# Crop the choosen filepath to only the name
	#_skeleton_filename_ = _skeleton_filename_.split('/')[1].split('.')[0]

	# Step trough the list of known files with missed data.
	for name in _list_of_missed_skeletons_:
		# Remove the trialing backslash from this name in the list.
		name = name.rstrip('\n')

		#if the name is in the list return true
		if _skeleton_filename_ == name:
			return True
	# otherwise return false		
	return False

# ----------------------------------------------------------------------------------------------------------------------------------------------------------

# Parse the command line arguments
def parseOpts( argv ):

	skeleton_name = ""

	# generate parser object
	parser = argparse.ArgumentParser()
	# add arguments to the parser so he can parse the shit out of the command line
	parser.add_argument("-path", "--path_name", action='store', dest='path_name', help="The path to the dataset.")
	parser.add_argument("-pn", "--part_name", action='store', dest='dataset_name', help="The name of the dataset in the path.")
	parser.add_argument("-aL", "--action_list", action='store', dest='action_list', help="A list of actions in the form: -aL A001,A002,A003,...  ")
	parser.add_argument("-iT", "--ignore_tail", action='store_true', dest='ignore_tail', default='False', help="The last frame in each set will be skipped if this flag is enabled." )
	parser.add_argument("-v", "--verbose", action='store_true', dest='verbose', default='False', help="True if you want to listen to the chit-chat.")

	# finally parse the command line 
	args = parser.parse_args()

	print("\n\nInformation: ---------------------------------------------------------------------------------------------------------------")

	# If a specific path is given
	if( args.path_name ):
		path_name = args.path_name
	else:		
		path_name = "skeleton/"

	# If a dataset name is given ( the list of datasets to compute then starts at this position )
	if( args.dataset_name ):
		if( args.dataset_name == "*"):
			skeleton_name = "S001C001P001R001A001.skeleton"
		else:
			skeleton_name = args.dataset_name + ".skeleton"
	else: 
		skeleton_name = "S001C001P001R001A001.skeleton"
		print ("\nNo dataset defined. Falling back to default dataset: "+ skeleton_name + " at location: "+ path_name )

	if args.action_list:
		action_list = args.action_list.split(",")
	else:
		action_list = None

	print ("\nConfiguration:")
	print ("----------------------------------------------------------------------------------------------------------------------------")
	print ("Path         : ", path_name)
	print ("Set          : ", skeleton_name)
	print ("Action list  : ", action_list)
	print ("Ignore tail  : ", args.ignore_tail)
	print ("verbose      : ", args.verbose)
	print ("\n\n")

	return path_name, skeleton_name, action_list, args.ignore_tail, args.verbose

# ----------------------------------------------------------------------------------------------------------------------------------------------------------

if __name__ == '__main__':
	main()
