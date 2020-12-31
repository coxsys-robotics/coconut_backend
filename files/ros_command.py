import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import InteractiveMarkerInit


from std_srvs.srv import SetBool

import math

'''

'''
class ros_talker():

    def __init__(self):
        self.msg = Twist()
        rospy.init_node('interface_command', anonymous=True)
    
        ### manual control ###
        self.cmd_topic_name = 'cmd_vel'
        self.cmd_vel_pub = rospy.Publisher(self.cmd_topic_name, Twist, queue_size=10)
        self.fork_srv = rospy.ServiceProxy('set_fork',SetBool)  
        ### ### ###

        ### robot status ###
        self.odom_topic_name = 'odom'
        self.position = [0,0]
        self.orientation = [0,0,0,1]
        self.odom_sub = rospy.Subscriber(self.odom_topic_name, Odometry, self.odom_callback)
        ### ### ###

        ### to goal ###
        self.goal_topic_name = '/move_base_simple/goal'
        self.goal = PoseStamped()
        self.goal_pub = rospy.Publisher(self.goal_topic_name, PoseStamped, queue_size=10)
        ### ### ###

        ### via points ###
        self.viapoint_topic = '/path_memorizer/update_full'
        self.markers = InteractiveMarkerInit()
        self.viapoints = []
        self.viapoint_sub = rospy.Subscriber(self.viapoint_topic, InteractiveMarkerInit, self.marker_callback)


    def movement_command(self,x,a):
        self.msg.linear.x = x
        self.msg.angular.z = a
        self.cmd_vel_pub.publish(self.msg)

    def soft_brake(self):
        self.msg.linear.x = 0
        self.msg.angular.z = 0
        self.cmd_vel_pub.publish(self.msg)

    def odom_callback(self, data):
        self.position[0] = data.pose.pose.position.x
        self.position[1] = data.pose.pose.position.y
        q = data.pose.pose.orientation
        self.orientation = [q.x, q.y, q.z, q.w] 
        
    def set_goal(self,x,y,q):
        self.goal.header.frame_id = "map"
        self.goal.header.stamp = rospy.Time.now()
        self.goal.pose.position.x = x
        self.goal.pose.position.y = y
        self.goal.pose.orientation = Quaternion(q['x'], q['y'], q['z'], q['w'])

        self.goal_pub.publish(self.goal)

    def marker_callback(self,data):
        self.viapoints = []
        for point in data.markers :
            if(point.controls[0].markers[0].color.g==0):
                self.viapoints.append(point.pose)


# movebase goal topic :  /move_base_simple/goalpoint