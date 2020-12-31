#!/usr/bin/env python

"""
After finish SLAM, map will save as <map_name>.yaml and <map_name>.pgm
This code will copy files to <map_name>_nav.yaml and <map_name>_nav.pgm to use in navigation.
"""
#TODO: use better way to get file path for yaml_path and image_path. Not fixed path like this.

import rospy
from shutil import copyfile
from os.path import expanduser

def create_nav_yaml(yaml_file, new_yaml_file):
	copyfile(yaml_file, new_yaml_file)

def write_nav_yaml(new_yaml_file, yaml_path, map_name):
	file = open(new_yaml_file, "r")
	lines = file.readlines()
	lines[0] = "image: {}{}_nav.pgm\n".format(yaml_path, map_name)
	file = open(new_yaml_file, "w")
	file.writelines(lines)
	file.close()

def create_nav_image(image_file, new_image_file):
	copyfile(image_file, new_image_file)

def copy_nav_map_yaml_main():
	home = expanduser("~")
	map_name = rospy.get_param("/map_name", 'test')
	# map_name = "fortuna_4th_floor"
	
	# NAV YAML
	yaml_path = "{}/catkin_ws/src/coconut_bringup/map/".format(home)
	yaml_filename = "{}.yaml".format(map_name)
	yaml_file = yaml_path + yaml_filename

	new_yaml_filename = "{}_nav.yaml".format(map_name)
	new_yaml_file = yaml_path + new_yaml_filename

	create_nav_yaml(yaml_file, new_yaml_file)
	write_nav_yaml(new_yaml_file ,yaml_path, map_name)
	
	# NAV IMAGE
	image_path = "{}/catkin_ws/src/coconut_bringup/map/".format(home)
	image_filename = "{}.pgm".format(map_name)
	image_file = image_path + image_filename

	new_image_filename = "{}_nav.pgm".format(map_name)
	new_image_file = image_path + new_image_filename

	create_nav_image(image_file, new_image_file)


