##############################
'''

This server use socket 
will handle continuous connection and messages that robot need to send first such as
   manual control, robot status, goal status.
'''
##############################

import time

from flask import Flask, render_template
from flask_socketio import SocketIO, emit, disconnect

from ros_for_socket import ros_talker #, ros_exit_exception

import json
import yaml

### time.sleep to wait a bit for ROS to start first after turn on computer 
time.sleep(15)

### Open config yaml file
config_folder = "config"
config_name = "config"

with open(config_folder + "/" + config_name + ".yaml", 'r') as stream:
    config_data = yaml.safe_load(stream)

app = Flask(__name__)
#app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app,logger=True,  engineio_logger=True)

ip = config_data['ip'] 
port = config_data['socket_port']  


### Create instance of class to communicate with ROS
ros_talker = ros_talker( config_data ) 


@socketio.on('connect')
def on_connect():
    socketio.emit('message',"connected")
    
@socketio.on('disconnect')
def on_disconnect():
    socketio.emit('message',"disconnected")
    ros_talker.movement_command(0,0)  ##stop robot if socket got disconnected.

@socketio.on('message')
def handle_message(message):
    print('received message: ' + message)


################  EVENT FOR MANUAL CONTROL  ################

'''
receive movement data as dict
"x": linear_velocity (m/s) #note wheel velocity = robot velocity when steering angle is 0
"z": angular velocity (rad/s)
'''
@socketio.on('movement')
def on_movement(move_json):
    print(str(move_json['x']) + ", " + str(move_json['z']))
    ros_talker.movement_command(move_json['x'],move_json['z'])


##############################################################
################  REQUEST POSITION EVENT  #################

'''
emit position object 
x: position x of robot (metre)
y: position y of robot (metre)
q: orientation of robot (quaternion (x,y,z,w))
'''
@socketio.on('pos')
def on_pos():
    #print("pos")
    obj = {}
    obj['x'] = ros_talker.position[0]
    obj['y'] = ros_talker.position[1]
    obj['q'] = ros_talker.orientation
    emit("pos", obj)

###########################################################
################  REQUEST ROBOT STATUS  #################

'''
emit status object 
uvc: whether uvc is ON or OFF. (bool)
goal: [x,y] of new goal. send only when new value received. (both are in metre) 
state: current workflow state. send only when new value received. (String)
'''
"""
coconut_state, String
    localize
    get_robot_initial_position
    move_to_goal
    cleaning
    move_to_initial
    initial_alignment
    uvc_workflow_done
"""
@socketio.on('status')
def on_status():
    #print("pos")
    obj = {}
    # obj['bat'] = ros_talker.battery
    obj['uvc'] = ros_talker.uvc_state
    if(ros_talker.new_goal):
        obj['goal'] = ros_talker.next_goal
        ros_talker.new_goal = False
    if(ros_talker.new_state):
        obj['state'] = ros_talker.current_state    
        ros_talker.new_state = False 
    emit("status", obj)

###########################################################
################# REQUEST BATTERY STATUS ##################
"""
emit battery status
"""
@socketio.on('battery')
def on_battery():
    obj = {}
    obj['bat'] = ros_talker.battery
    emit('battery',obj)

###########################################################
################# RECIEVE TASK #################

@socketio.on('task')
def on_task(tasks_json):
    pass

@socketio.on('goal')
def on_goal(goal_json):
    # print("-----")
    # print(goal_json['goals'])
    # print("-----")
    # if len(goal_json['goals'])==1:
    #     goal = goal_json['goals'][0]
    #     ros_talker.set_goal(goal['x'], goal['y'], goal['q'])
    # else:
    #     ros_talker.set_path(goal_json['goals'])

    ros_talker.set_path(goal_json['goals'])

################################################

@socketio.on('mock')
def on_mock():
    ros_talker.mock()

@socketio.on('talk')
def on_talk():
    print("talk")
    emit("talk_back")



if __name__ == '__main__':
    try:
        # ros_talker.start()
        # ros_talker.neopixel_start(3)

        socketio.run(app,host=ip, port=port, debug=False)
        
    except KeyboardInterrupt:
        exit()

    # except ros_exit_exception:
    #     exit()