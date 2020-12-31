#!/usr/bin/env python

"""
Read .yaml config file from a folder  
Receive new value and write to .yaml file
  for cartographer SLAM
  adjust 4 value:
	TRAJECTORY_BUILDER_2D.submaps.num_range_data
	TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight
	TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight
	POSE_GRAPH.optimize_every_n_nodes"
"""
#TODO: use better way to get file path for luaSlam_path and luaLocalization_path. Not fixed path like this.

import rospy
from os.path import expanduser

word_1 = "TRAJECTORY_BUILDER_2D.submaps.num_range_data"
word_2 = "TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight"
word_3 = "TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight"
word_4 = "POSE_GRAPH.optimize_every_n_nodes"

def read_slam_lua(filename):
	num_range_data = 0
	translation_weight = 0
	rotation_weight = 0
	
	slam_line = 0
	num_range_data_line = 0
	translation_weight_line = 0
	rotation_weight_line = 0

	file = open(filename)
	for line in file:
		slam_line = slam_line + 1
		line.strip().split('/n')
		if word_1 in line:
			num_range_data = line[line.find(" =")+3:]
			num_range_data = int(num_range_data.split('--')[0])
			num_range_data_line = slam_line
		if word_2 in line:
			translation_weight = line[line.find(" =")+3:]
			translation_weight = int(translation_weight.split('--')[0])
			translation_weight_line = slam_line
		if word_3 in line:
			rotation_weight = line[line.find(" =")+3:]
			rotation_weight = int(rotation_weight.split('--')[0])
			rotation_weight_line = slam_line
			break

	print("Num range data: %d at line %d" %(num_range_data, num_range_data_line))
	print("Translation weight: %d at line %d" %(translation_weight, translation_weight_line))
	print("Rotation weight: %d at line %d " %(rotation_weight, rotation_weight_line))

	return num_range_data, num_range_data_line, translation_weight, translation_weight_line, rotation_weight, rotation_weight_line

def write_slam_lua(filename, user_num_range_data, num_range_data_line, user_translation_weight, translation_weight_line, user_rotation_weight, rotation_weight_line):
	file = open(filename, "r")
	lines = file.readlines()
	lines[num_range_data_line-1] = "TRAJECTORY_BUILDER_2D.submaps.num_range_data = {}-- 160\n".format(user_num_range_data)
	lines[translation_weight_line-1] = "TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = {} --1 -- 10\n".format(user_translation_weight)
	lines[rotation_weight_line-1] = "TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = {} --270 -- 40\n".format(user_rotation_weight)

	file = open(filename, "w")
	file.writelines(lines)
	file.close()


def read_localization_lua(filename):
	optimize_every_n_nodes = 0

	localization_line = 0
	optimize_every_n_nodes_line = 0

	file = open(filename)
	for line in file:
		localization_line = localization_line + 1
		line.strip().split('/n')
		if word_4 in line:
			optimize_every_n_nodes = line[line.find(" =")+3:]
			optimize_every_n_nodes = int(optimize_every_n_nodes.split('--')[0])
			optimize_every_n_nodes_line = localization_line
			break
	
	print("Optimize every n node: %d at line %d" %(optimize_every_n_nodes, optimize_every_n_nodes_line))
	
	return optimize_every_n_nodes, optimize_every_n_nodes_line

def write_locaizaition_lua(filename, user_optimize_every_n_nodes, optimize_every_n_nodes_line):
	file = open(filename, "r")
	lines = file.readlines()
	lines[optimize_every_n_nodes_line-1] = "POSE_GRAPH.optimize_every_n_nodes = {} --90\n".format(user_optimize_every_n_nodes)

	file = open(filename, "w")
	file.writelines(lines)
	file.close()

def main(user_num_range_data = 59, user_translation_weight = 2, user_rotation_weight = 43, user_optimize_every_n_nodes = 2):
	home = expanduser("~")

	lua_slam_name = rospy.get_param("/map_name", 'test')
	lua_localization_name = rospy.get_param("/map_name", 'test')

	# user_num_range_data = 59
	# user_translation_weight = 2
	# user_rotation_weight = 43
	# user_optimize_every_n_nodes = 2

	luaSlam_path = "{}/catkin_ws/src/coconut_bringup/config/".format(home)
	luaSlam_filename = "{}_slam.lua".format(lua_slam_name)
	luaSlam_file = luaSlam_path + luaSlam_filename

	luaLocalization_path = "{}/catkin_ws/src/coconut_bringup/config/".format(home)
	luaLocalization_filename = "{}_localization.lua".format(lua_localization_name)
	luaLocalization_file = luaLocalization_path + luaLocalization_filename

	num_range_data, num_range_data_line, translation_weight, translation_weight_line, rotation_weight, rotation_weight_line = read_slam_lua(luaSlam_file)
	if num_range_data != user_num_range_data or translation_weight != user_translation_weight or rotation_weight != user_rotation_weight:
		write_slam_lua(luaSlam_file, user_num_range_data, num_range_data_line, user_translation_weight, 
		translation_weight_line, user_rotation_weight, rotation_weight_line)

	optimize_every_n_nodes, optimize_every_n_nodes_line = read_localization_lua(luaLocalization_file)
	if optimize_every_n_nodes != user_optimize_every_n_nodes:
		write_locaizaition_lua(luaLocalization_file, user_optimize_every_n_nodes, optimize_every_n_nodes_line)

	print("\nNum range data: %d at line %d" %(user_num_range_data, num_range_data_line))
	print("Translation weight: %d at line %d" %(user_translation_weight, translation_weight_line))
	print("Rotation weight: %d at line %d " %(user_rotation_weight, rotation_weight_line))
	print("Optimize every n node: %d at line %d" %(user_optimize_every_n_nodes, optimize_every_n_nodes_line))


if __name__ == '__main__':
	main()