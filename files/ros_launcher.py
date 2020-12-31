"""
Receive command from restful api and launch roslaunch using python roslaunch module
"""
#TODO: use better way to get/assign launch file's path. Not use fixed path like this.

import rospy
import roslaunch
from os.path import expanduser
# import threading

class ros_launcher():

    ### init parameters
    def __init__(self):
        # threading.Thread(target=lambda: rospy.init_node('node_to_call_roslaunch', disable_signals=True)).start()
        
        self.loop_save_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.online_slam_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.offline_slam_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.save_map_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        
        home = expanduser("~")
        self.path_to_loop_save = '{}/catkin_ws/src/coconut_slam/launch/loop_save_map.launch'.format(home)
        self.path_to_online_slam = '{}/catkin_ws/src/coconut_bringup/launch/coconut_slam.launch'.format(home)
        self.path_to_offline_slam = '{}/catkin_ws/src/coconut_bringup/launch/offline_slam.launch'.format(home)
        self.path_to_save_map = '{}/catkin_ws/src/coconut_slam/launch/save_map.launch'.format(home)

        self.parent_loop_save = None
        self.parent_online_slam = None
        self.parent_offline_slam = None
        self.parent_save_map = None

        self.continue_save_map = False
        self.loop_map_name = ""

    ####### function to start and shutdown loop_save_map.launch #######
    ### have frontend call this function every n second instead
    ### have it stop when run save_map.launch
    def launch_loop_save(self, filename):
        self.loop_map_name = filename
        cli_args = [self.path_to_loop_save, "filename:={}".format(filename)]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

        self.parent_loop_save = roslaunch.parent.ROSLaunchParent(self.loop_save_uuid, roslaunch_file)

        self.parent_loop_save.start()
        self.parent_loop_save.shutdown()

    # def shutdown_loop_save(self):
    #     self.continue_save_map = False

    ##########################################################
    ####### function to start and shutdown online_slam #######

    def launch_online_slam(self, bag_name, lua_name):
        cli_args = [self.path_to_online_slam, "bag_name:={}".format(bag_name), "lua_name:={}".format(lua_name)]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

        self.parent_online_slam = roslaunch.parent.ROSLaunchParent(self.online_slam_uuid, roslaunch_file)

        self.parent_online_slam.start()

    def shutdown_online_slam(self):
        self.parent_online_slam.shutdown()

    ##################################################
    ####### function to launch save_map.launch #######

    def launch_save_map(self, filename):
        cli_args = [self.path_to_save_map, "filename:={}".format(filename)]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

        self.parent_save_map = roslaunch.parent.ROSLaunchParent(self.save_map_uuid, roslaunch_file)

        self.parent_save_map.start()
        rospy.sleep(5)
        self.parent_save_map.shutdown()


    ###########################################################
    ####### function to start and shutdown offline_slam #######

    def launch_offline_slam(self, bag_name, lua_name):
        cli_args = [self.path_to_offline_slam, "bag_name:={}".format(bag_name), "lua_name:={}".format(lua_name)]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

        self.parent_offline_slam = roslaunch.parent.ROSLaunchParent(self.offline_slam_uuid, roslaunch_file)

        self.parent_offline_slam.start()

    def shutdown_offline_slam(self):
        self.parent_offline_slam.shutdown()