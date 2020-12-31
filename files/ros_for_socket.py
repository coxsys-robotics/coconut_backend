import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose, Quaternion
from actionlib_msgs.msg import GoalStatusArray
from os.path import expanduser
import math
import time

import threading

from std_srvs.srv import SetBool

from std_msgs.msg import String, Float64, Bool, UInt8, UInt8MultiArray, UInt16, UInt16MultiArray

from generate_report import report_generator

home = expanduser("~")

class ros_talker():

    def __init__(self, config_data ): # cmd_topic='tablet_vel_cmd_vel', position_topic='odom', goal_topic='/move_base_simple/goal', goal_status_topic='/move_base/status'):
        threading.Thread(target=lambda: rospy.init_node('data_for_flask_socket', disable_signals=True)).start()
        # rospy.init_node('data_for_flask_socket')
    
        ### manual control ###
        self.cmd_topic_name = config_data['cmd_topic'] #cmd_topic
        self.cmd_msg = Twist()
        self.cmd_vel_pub = rospy.Publisher(self.cmd_topic_name, Twist, queue_size=10)
        ### ### ###

        ### robot status ###
        self.position_topic_name = config_data['position_topic'] #position_topic
        self.position = [0,0]
        self.orientation = [0,0,0,1]
        self.position_sub = rospy.Subscriber(self.position_topic_name, Pose, self.position_callback)
        ### ### ###

        ### to goal ###
        self.goal_send = False
        self.goal_reach = False
        self.wait_result = False
        self.goal = PoseStamped()
        self.goals_list = []
        self.command = ""
        self.goal_topic_name = config_data['goal_topic'] #goal_topic
        self.goal_status_topic_name = config_data['move_base_status'] #goal_status_topic
        self.goal_pub = rospy.Publisher(self.goal_topic_name, PoseStamped, queue_size=10)
        self.goat_stat_sub = rospy.Subscriber(self.goal_status_topic_name, GoalStatusArray, self.goal_status_callback)
        ### ### ###

        ### uvc command ###
        self.uvc_msg = Bool()
        self.uvc_pub = rospy.Publisher("/cmd_uvc",Bool,queue_size=10)
        self.uvc_state_sub = rospy.Subscriber("/device_status", UInt8, self.uvc_callback)
        self.uvc_state = False
        self.uvc_start_time = 0
        ### ### ###

        ### work flow state ###
        self.next_goal = [0,0]
        self.current_state = ""
        self.new_goal = False
        self.new_state = False
        self.current_goal_sup = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)
        self.work_state_sub = rospy.Subscriber("/coconut_state", String, self.work_state_callback)
        self.gui_cmd_sub = rospy.Subscriber("/gui_cmd", String, self.gui_cmd_callback)
        ### ### ###

        ### battery ###
        self.battery = 0
        self.bat_sub = rospy.Subscriber("/battery_percent", UInt8, self.bat_callback)
        ### ### ###

        ### forklift command ###
        self.fork_pub = rospy.Publisher("/pallet/hub_to_lift_Joint_position_controller/command",Float64,queue_size=10)
        ### ### ###

        ### report generator ###
        self.report_generator = report_generator(map_folder=home + config_data['map_folder'], vertices_folder=config_data['vertice_save_directory'],
                        edges_folder=config_data['edge_save_directory'], zones_folder=config_data['zone_save_directory'], pdf_path=home+'/'+config_data['report_directory'])
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

        # rospy.on_shutdown(self.on_shutdown)

    def movement_command(self,x,a):
        self.cmd_msg.linear.x = x
        self.cmd_msg.angular.z = a
        self.cmd_vel_pub.publish(self.cmd_msg)

    def soft_brake(self):
        self.cmd_msg.linear.x = 0
        self.cmd_msg.angular.z = 0
        self.cmd_vel_pub.publish(self.cmd_msg)

    def position_callback(self, data):
        self.position[0] = data.position.x
        self.position[1] = data.position.y
        q = data.orientation
        self.orientation = [q.x, q.y, q.z, q.w] 

    def neopixel_start(self,sleep_time):
        time.sleep(sleep_time)
        self.neopixel_command(0,30,30,100)


    def neopixel_command(self, mode=0, r=100, g=100, b=100):
        self.neopixel_rgb_msg.data = [ r, g, b]
        self.neopixel_rgb_publisher.publish(self.neopixel_rgb_msg)
        self.neopixel_mode_msg.data = mode
        self.neopixel_mode_publisher.publish(self.neopixel_mode_msg)
    
    def uvc_publish(self, data):
        self.uvc_msg.data = data
        self.uvc_pub.publish(self.uvc_msg)

    def uvc_callback(self,data):
        if(data.data&128==128):
            self.uvc_state = True
        else:
            self.uvc_state = False

    def gui_cmd_callback(self,data):
        if(data.data=='start'):
            self.uvc_start_time = time.time()
            self.report_generator.load_report_data(rospy.get_param('map_name') + '_nav')
            self.report_generator.draw_image(rospy.get_param('uvc_command'))

    def work_state_callback(self,data):
        self.new_state = True
        self.current_state = data.data
        if(data.data=='uvc_workflow_done'):
            used_time = time.time() - self.uvc_start_time
            self.report_generator.report_data.set_used_time(used_time)
            self.report_generator.generateReport()

    def goal_callback(self,data):
        self.new_goal = True
        self.next_goal = [data.pose.position.x, data.pose.position.y]


    def bat_callback(self,data):
        self.battery = data.data


    """
    publish 1 goal with position x,y and quaternion q
    """
    def set_goal(self,x,y,q):
        self.goal.header.frame_id = "map"
        self.goal.header.stamp = rospy.Time.now()
        self.goal.pose.position.x = x
        self.goal.pose.position.y = y
        self.goal.pose.orientation = Quaternion(q['x'], q['y'], q['z'], q['w'])

        self.goal_reach = False
        self.goal_pub.publish(self.goal)
        print("**** goal send ****")

    """
    send goal with same postion ascurrent robot position to make robot stop
    """
    def stop_goal(self):
        self.goal.header.frame_id = "map"
        self.goal.header.stamp = rospy.Time.now()
        self.goal.pose.position.x = self.position[0]
        self.goal.pose.position.y = self.position[1]
        self.goal.pose.orientation = Quaternion(self.orientation[0],self.orientation[1],self.orientation[2],self.orientation[3])

        self.goal_pub.publish(self.goal)
        print("**** goal stop ****")


    def set_path(self, goals_list):
        self.goals_list = goals_list.copy()
        self.set_goal(self.goals_list[0]['x'], self.goals_list[0]['y'], self.goals_list[0]['q'])
        self.command = self.goals_list[0]['command']
        self.goals_list.pop(0)
        self.wait_result = True
        self.goal_send = False
        self.goal_reach = False


    def goal_status_callback(self, data):
        """
        receive move base goal status.
        status:
        1: goal accepted
        2: goal canceled by another goal
        3: goal reach
        4: fail to reach goal
        """
        if(self.wait_result):
            if (len(data.status_list)>0):
                if(data.status_list[len(data.status_list)-1].status==1 and not self.goal_reach):
                    self.goal_send = True
                elif(data.status_list[len(data.status_list)-1].status==3 and self.goal_send):
                    self.goal_reach = True
                    print("goal reach")
                    if(self.command!=""):
                        rospy.sleep(1)
                    if(len(self.goals_list)>0):
                        print("send new goal")
                        self.set_goal(self.goals_list[0]['x'], self.goals_list[0]['y'], self.goals_list[0]['q'])
                        self.command = self.goals_list[0]['command']
                        self.goals_list.pop(0)
                    else:
                        self.wait_result = False
                    self.goal_send = False
                    self.goal_reach = False
