##############################
'''
Communicate with ros. Get command and data from flask-restful
'''
##############################

import rospy
import threading
from os.path import expanduser

from geometry_msgs.msg import Pose
from visualization_msgs.msg import InteractiveMarkerInit

from std_msgs.msg import String, Float64, Bool, UInt8, UInt8MultiArray, UInt16, UInt16MultiArray

from create_lua_file import main

from pgm_to_png import convert_image

import time

home = expanduser("~")


class ros_talker():

    def __init__(self,config_data):

        threading.Thread(target=lambda: rospy.init_node('data_for_flask_restful', disable_signals=True)).start()
        # rospy.init_node('data_for_flask_restful')

        ### robot status ###
        self.position_topic_name = 'pose_from_robot'
        self.position = [0,0]
        self.orientation = [0,0,0,1]
        self.position_sub = rospy.Subscriber(self.position_topic_name, Pose, self.position_callback)
        ### ### ###

        ### Initialize robot pose ###
        self.initial_pose_pub = rospy.Publisher("app_initial_pose", Pose, queue_size=10)
        ### ### ###

        ### Neopixel (read manual at https://docs.google.com/spreadsheets/d/1sDBpzWSkR-TJ_CMlEihfyFtKzg1GNOJjjiK5VN6d-MU/edit#gid=0) ###
        ## neopixel mode ##
        self.neopixel_mode_msg = UInt8()
        self.neopixel_mode_publisher = rospy.Publisher("neopixel_mode", UInt8, queue_size=10)
        ## neopixel rgb : [r0,g0,b0, r1,g1,b1, r2,g2,b2] ##
        self.neopixel_rgb_msg = UInt8MultiArray()
        self.neopixel_rgb_publisher = rospy.Publisher("neopixel_rgb", UInt8MultiArray, queue_size=10)
        ## neopixel time ##
        self.neopixel_time_msg = UInt16MultiArray()
        self.neopixel_time_publisher = rospy.Publisher("neopixel_time_ms", UInt16MultiArray, queue_size=10)
        ## neopixel number of led ##
        self.neopixel_num_msg = UInt16MultiArray()
        self.neopixel_num_publisher = rospy.Publisher("neopixel_num", UInt16MultiArray, queue_size=10)
        ## neopixel number of step in run mode ##
        self.neopixel_step_msg = UInt16()
        self.neopixel_step_publisher = rospy.Publisher("neopixel_step", UInt16, queue_size=10)
        ### ### ###

        ### COCONUT STATE ###
        self.coconut_state_sub = rospy.Subscriber("coconut_state", String, self.coconut_state_callback)
        self.map_folder = home + config_data['map_folder']
        ### ### ###

        self.uvc_command_pub = rospy.Publisher("gui_cmd", String, queue_size=10)
        self.delivery_command_pub = rospy.Publisher("gui_cmd", String, queue_size=10)

        self.call_launch_pub = rospy.Publisher("call_launch",String, queue_size=10)

    def position_callback(self, data):
        self.position[0] = data.position.x
        self.position[1] = data.position.y
        q = data.orientation
        self.orientation = [q.x, q.y, q.z, q.w] 

    def get_param(self, name):
        if(rospy.has_param(name)):
            return rospy.get_param(name)

    def set_param(self,name, value, force=False):
        if(rospy.has_param(name) or force):
            rospy.set_param(name,value)

    def neopixel_start(self,sleep_time):
        time.sleep(sleep_time)
        self.neopixel_command(0,30,30,100)


    def neopixel_command(self, mode=0, r=100, g=100, b=100):
        self.neopixel_rgb_msg.data = [ r, g, b]
        self.neopixel_rgb_publisher.publish(self.neopixel_rgb_msg)
        self.neopixel_mode_msg.data = mode
        self.neopixel_mode_publisher.publish(self.neopixel_mode_msg)

    def coconut_state_callback(self,data):
        if(data.data=="save_map_shutdown"):
            map_name = self.get_param("map_name")
            convert_image(self.map_folder + '/' + map_name + '.pgm', self.map_folder)

    def uvc_start(self):
        msg = String()
        msg.data = "start"
        self.uvc_command_pub.publish(msg)

    def uvc_pause(self):
        msg = String()
        msg.data = "pause"
        self.uvc_command_pub.publish(msg)

    def uvc_continue(self):
        msg = String()
        msg.data = "continue"
        self.uvc_command_pub.publish(msg)

    def set_delivery_goal(self, command):
        rospy.set_param('/rooms_to_deliver',command)

    def delivery_start(self):
        msg = String()
        msg.data = "start"
        self.delivery_command_pub.publish(msg)

    def initial_pose_publish(self,x,y,z,w):
        pose_msg = Pose()
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.orientation.z = z
        pose_msg.orientation.w = w
        self.initial_pose_pub.publish(pose_msg)

    """
    call_launch String
    online_slam ( + _shutdown)
    offline_slam ( + _shutdown)
    loop_save_map ( + _shutdown)
    robot ( + _shutdown)
    """
    def call_launch(self, command):
        msg = String()
        msg.data = command    
        self.call_launch_pub.publish(msg)
    
    def create_lua(self):
        main()

    # def start(self):
    #     #self.node = rospy.init_node('data_for_flask_restful')
    #     rospy.init_node('data_for_flask_restful')
