##############################
'''
use flask restful
frontend app will send a GET request to this program.
'''
#TODO: Add doc for json structure for each command.
##############################

import time

from flask import Flask, send_file, request
from flask_restful import Resource, Api, reqparse
import werkzeug, os, glob, shutil
from os.path import expanduser

import yaml
import json

import numpy as np
import math
import random

from pgm_to_png import convert_image

from ros_for_restful import ros_talker
from filter_text_file import main
from ros_launcher import ros_launcher
import adjust_slam_parameter
from edit_map_nav import edit_map_main
from copy_nav_map_yaml import copy_nav_map_yaml_main

### time.sleep to wait a bit for ROS to start first after turn on computer 
time.sleep(15)

home = expanduser("~")

### Open config yaml file
config_folder = "config"
config_name = "config"
param_config_name = "param_config"

with open(config_folder + "/" + config_name + ".yaml", 'r') as stream:
    config_data = yaml.safe_load(stream)

with open(config_folder + "/" + param_config_name + ".yaml", 'r') as stream:
    param_config_data = yaml.safe_load(stream)
### ### ###


app = Flask(__name__)
api = Api(app)


### Get config value
ip = config_data['ip']
port = config_data['restful_port'] 

map_folder = home + config_data['map_folder']
map_name = "" 
map_file_format = config_data['map_file_format']

room_data_save_directory = config_data['room_save_directory']

vertice_data_save_directory = config_data['vertice_save_directory']
edge_data_save_directory = config_data['edge_save_directory']
zone_data_save_directory = config_data['zone_save_directory']

delivery_param_name = config_data['delivery_param']

deli_text_file = home + config_data['deli_text']


### Create instance of class to communicate with ROS
ros_talker = ros_talker( config_data )
launcher = ros_launcher()
ros_talker.set_param("/map_name", map_name, True)

parser = reqparse.RequestParser()
parser.add_argument('file',type=werkzeug.datastructures.FileStorage, location='files')


class HelloWorld(Resource):
    """
    just a test page. access <robot's ip>:5556 and see if this information pop up or not.
    """
    def get(self):
        return {'/'           : 'home', 
                '/maps'      : 'list of maps', 
                '/image'      : 'map image', 
                '/res'        : 'map data from .yaml', 
                '/point'      : 'viapoints',
                '/setting'    : 'setting',
                '/save_room'  : 'saved rooms',
                '/save_vertice'  : 'saved vertices',
                '/save_edge'  : 'saved edges',
                '/save_zone'  : 'saved zones',
                '/delivery_command' : 'send delivery command',
                '/edit_map'    : 'edit_map',
                '/slam'        : 'slam'}


class choose_map(Resource):
    """
    get: return all avaiable maps in map_folder. 
    post: {'name':'<choosen map name>'} choose the map that want to open
    """
    def get(self):
        filelist = glob.glob(map_folder + "/*." + map_file_format)
        namelist = []
        for f in filelist:
            namelist.append( os.path.basename(f))
        return namelist
    def post(self):
        global map_name
        json_data = request.get_json(force=True)
        print(json_data)
        map_name =  os.path.splitext(json_data['name'])[0]
        ros_talker.set_param("/map_name", map_name, True)
        return

class map_image(Resource):
    """
    *** must set map_name first. ***
    return map file. map's name is set in variable map_name
    """
    def get(self):
        filename = map_folder + '/' + map_name + '.' + map_file_format
        try:
            return send_file(filename, mimetype='image/png')
        except FileNotFoundError as exc:
            print(exc)
            return

class map_resolution(Resource):
    """
    *** must set map_name first. ***
    get and return map data from .yaml file
    """
    def get(self):
        filename = map_folder + '/' + map_name + '.yaml'
        try:
            with open(filename,'r') as stream:
                data = yaml.safe_load(stream)
                return data
        except yaml.YAMLError as exc:
            print(exc)
        except FileNotFoundError as exc:
            print(exc)
        return 

class setting(Resource):
    """
    get: get value from rosparam
    post: set value to rosparam
    """
    def get(self):
        to_return = {}
        to_return['parameters'] = []
        ### get all param in config from ros.
        for key in param_config_data:
            tmp_dict = {}
            tmp_dict["key"] = key
            tmp_dict["value"] = ros_talker.get_param(param_config_data[key])
            to_return['parameters'].append(tmp_dict)
        return to_return
    def post(self):
        json_data = request.get_json(force=True)  ## get_json() return dict type
        # print(type(json_data))
        # dict_data = json.loads(json_data)
        for key in json_data:
            ros_talker.set_param(param_config_data[key], json_data[key])

class save_room(Resource):
    """
    *** must set map_name first. ***
    get: get names of saved rooms in map from txt file.
    post: receive json data and save as rooms' names in text file
    """
    def get(self):
        # with open(room_data_save_directory + "/"+ map_name + ".yaml") as file:
        #     # The FullLoader parameter handles the conversion from YAML
        #     # scalar values to Python the dictionary format
        #     rooms = yaml.load(file, Loader=yaml.FullLoader)
        # return rooms
        to_read = open(room_data_save_directory + "/"+ map_name + ".txt","r")
        return to_read.read()
    def post(self):
        json_data = request.get_json(force=True)
        # print(json_data)
        to_save = open(room_data_save_directory + "/"+ map_name + ".txt","w")
        for k in json_data:
            to_save.write(k + ":" + str(json_data[k]['pos_x']) + ',' + str(json_data[k]['pos_y']) + ',' + 
                str(json_data[k]['Qz']) + ',' + str(json_data[k]['Qw']) + "\n")
            # print(k)
            # print(json_data[k])
        to_save.close()

class delivery_command(Resource):
    """
    get: not avaiable
    post: receive json for list of room to go. Then execute delivery sequence.
    """
    def get(self):
        return
    def post(self):
        json_data = request.get_json(force=True)
        file_path = deli_text_file
        shutil.copy(room_data_save_directory + "/"+ map_name + ".txt", file_path)
        ros_talker.set_delivery_goal(json_data["room_list"])
        ros_talker.delivery_start()
        print(json_data["room_list"])


class slam(Resource):
    """
    get: return current image of SLAM map
    post: if command is online_slam then execute online SLAM
          if command is offline_slam, filter rosbag then perform offline SLAM
    *** when save map will save as 2 maps, for localization and for navigation. 
    *** localization: <map_name>.pgm/.yaml, navigation: <map_name>_nav.pgm/.yaml
    """
    def get(self):
        try:
            # convert_image(map_folder + '/' + map_name + '.pgm', map_folder)
            filename = map_folder + '/' + map_name + '.png'
            return send_file(filename, mimetype='image/png')
        except Exception as exc:
            print(exc)
            return
    def post(self):
        json_data = request.get_json(force=True)
        try:
            if(json_data['command']=="online_slam"):
                ros_talker.create_lua()
            elif(json_data['command']=="filter_bag"):
                ros_talker.create_lua()
                ros_talker.call_launch(json_data['command'])
                return "Filtering rosbag"
            elif(json_data['command']=="robot" and len(map_name)>4):
                if(map_name[-4:]=="_nav"):
                    ros_talker.set_param("map_name", map_name[:-4])
            if('params' in json_data.keys()):
                # print(json_data['user_num_range_data'])
                adjust_slam_parameter.main(json_data['user_num_range_data'], json_data['user_translation_weight'],
                                json_data['user_rotation_weight'], json_data['user_optimize_every_n_nodes'])
            ros_talker.call_launch(json_data['command'])
            if(json_data['command']=="save_map"):
                t = time.time()
                while( time.time()-t<30.0 or not os.path.isfile(map_folder + '/' + map_name + '.pgm')):
                    continue
                if( os.path.isfile(map_folder + '/' + map_name + '.pgm') ):
                    convert_image(map_folder + '/' + map_name + '.pgm', map_folder)
                    copy_nav_map_yaml_main()
                    if(os.path.isfile(map_folder + '/' + map_name + '_nav.pgm')):
                        convert_image(map_folder + '/' + map_name + '_nav.pgm', map_folder)
                        return "Map saved successfully."
                    return "Normal Map saved. Nav Map not found."
                else:
                    return "Internal error occured..."
            return "Robot received command."

        except Exception as e:
            print(e)
            return str(e)

class edit_map(Resource):
    """
    post: receive json data for line and polygon to draw on map file using opencv
    """
    def post(self):
        json_data = request.get_json(force=True)
        ## edit map with received json
        try:
            nav_image_filename = edit_map_main(json_data)
            convert_image(map_folder + '/' + nav_image_filename, map_folder)
        except Exception as e:
            print(e)
            
class set_initial_pose(Resource):
    """
    post: get json data of robot position, then set it to robot. (may not work on some localization algorithm)
    """
    def post(self):
        json_data = request.get_json(force=True)
        ros_talker.initial_pose_publish(json_data['x'], json_data['y'], json_data['z'], json_data['w'])

class random_neopixel(Resource):
    """
    randomly change color of neopixel led
    """
    def get(self):
        ros_talker.neopixel_command(0,  random.randrange(30,100), random.randrange(30,100), random.randrange(30,100))
        return "neopixeled"

def yaw_to_qz_qw(yaw):
    qz = np.sin(yaw/2)
    qw = np.cos(yaw/2)
    return qz, qw

api.add_resource(HelloWorld, '/')
api.add_resource(choose_map,'/maps')
api.add_resource(map_image,'/image')
api.add_resource(map_resolution,'/res')
# api.add_resource(viapoint,'/point')
api.add_resource(setting,'/setting')
api.add_resource(save_room,'/save_room')
api.add_resource(delivery_command,'/delivery_command')
api.add_resource(slam,'/slam')
api.add_resource(edit_map,'/edit_map')
api.add_resource(set_initial_pose,'/set_initial_pose')
api.add_resource(random_neopixel,'/neopixel')

if __name__ == '__main__':
    try:
        print("start")
        ros_talker.neopixel_start(3)
        app.run(host=ip, port=port,debug=False)
    except KeyboardInterrupt:
        exit()