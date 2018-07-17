#!/usr/bin/env python

# This class and related functions are an abstraction of the turtlesim above ROS,
# i.e. it hides calls to services and publishers in a convenient python API

# to work, this assumes you started turtlesim_node:
# rosrun turtlesim turtlesim_node


import math,time,threading

try:
    import rospy
    from geometry_msgs.msg import Twist
except:
    raise Exception("\n\nROS needs to be installed and sourced. Please visit http://www.ros.org/\n\n")
    
try :
    from turtlesim.srv import Spawn
    from turtlesim.srv import TeleportAbsolute
    from turtlesim.srv import Kill
    from turtlesim.srv import SetPen
    from turtlesim.msg import Pose
    from std_srvs.srv import Empty
except :
    raise Exception("\n\nturtlesim.py requires turtlesim to be installed and running. Please visit http://wiki.ros.org/turtlesim\n\n")



_SPAWN_SERVICE = "spawn"
_VELOCITY_TOPIC = "cmd_vel"
_TELEPORT_SERVICE = "teleport_absolute"
_KILL_SERVICE = "kill"
_PERMANENT_TURTLE_NAME = "turtle1"
_POSE_TOPIC = "pose"
_SET_PEN_SERVICE = "set_pen"
_CLEAR_SERVICE = "clear"


def kill_turtle(turtle_name):

    try:
        rospy.init_node('kill_turtle_'+turtle_name, anonymous=True)
    except :
        pass
        
    global _KILL_SERVICE
    
    rospy.wait_for_service(_KILL_SERVICE)
    service = rospy.ServiceProxy(_KILL_SERVICE, Kill)
    service(turtle_name)


def clear():

    global _CLEAR_SERVICE
    
    rospy.wait_for_service(_CLEAR_SERVICE)
    service = rospy.ServiceProxy(_CLEAR_SERVICE, Empty)
    service()


    

class Turtlesim:

    
    def __init__(self,x,y,theta,pen_off=False):

        global _VELOCITY_TOPIC
        global _PERMANENT_TURTLE_NAME

        # creating the turtle
        self.name = self._spawn(x,y,theta)
            
        # removing trail if asked to
        if pen_off:
            self.set_pen_off()
            
        # velocity as last requested by user
        self._current_velocity = [0,0] # [linear x, angular z]

        # for setting the desired velocity of this turtle
        self._velocity_publisher = rospy.Publisher("/"+self.name+"/"+_VELOCITY_TOPIC,
                                                  Twist, queue_size=10)

        # for getting current position and velocity
        # as published by the turtle
        self._position = None
        self._velocity = None
        self._lock = threading.Lock()
        def pose_callback(msg):
            with self._lock:
                self._position = [msg.x,msg.y,msg.theta]
                self._velocity = [msg.linear_velocity,msg.angular_velocity]
        self._subscriber = rospy.Subscriber(self.name+"/"+_POSE_TOPIC,Pose,pose_callback)

        self.pen_color = None

        
    def _spawn(self,x,y,theta):

        global _SPAWN_SERVICE
        
        rospy.wait_for_service(_SPAWN_SERVICE)
        service = rospy.ServiceProxy(_SPAWN_SERVICE, Spawn)
        response = service(x, y, theta, None)

        return response.name

    
    def set_pen(self,r,g,b,width):

        global _SET_PEN_SERVICE
        
        rospy.wait_for_service(self.name+"/"+_SET_PEN_SERVICE)
        service = rospy.ServiceProxy(self.name+"/"+_SET_PEN_SERVICE, SetPen)
        response = service(r, g, b, width, 0)

        self.pen_color = [r,g,b]

        
    def set_pen_off(self):

        global _SET_PEN_SERVICE
        
        rospy.wait_for_service(self.name+"/"+_SET_PEN_SERVICE)
        service = rospy.ServiceProxy(self.name+"/"+_SET_PEN_SERVICE, SetPen)
        response = service(0, 0, 0, 0, 1)
        
        
    def get_position(self):

        with self._lock:

            if self._position is None:
                return None

            return [p for p in self._position]

        
    def get_velocity(self):
        
        with self._lock:

            if self._velocity is None:
                return None

            return [v for v in self._velocity]

        
    def get_name(self):

        return self.name

    
    def _init_msg(self):

        msg = Twist()
        msg.linear.x = self._current_velocity[0]
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = self._current_velocity[1]

        return msg
        
        
    def set_velocity(self,x=None,angular=None):

        msg = self._init_msg()

        if x is not None:
            msg.linear.x = x
            self._current_velocity[0]=x
            
        if angular is not None:
            msg.angular.z = angular
            self._current_velocity[1]=angular

        self._velocity_publisher.publish(msg)

        
    def teleport(self,x,y,theta):

        global _TELEPORT_SERVICE
        
        rospy.wait_for_service(self.name+"/"+_TELEPORT_SERVICE)
        service = rospy.ServiceProxy(self.name+"/"+_TELEPORT_SERVICE, TeleportAbsolute)
        service(x, y, theta)

        
    def stop(self):

        self.set_linear_velocity(x=0,angular=0)

        
    def clear(self):

        global _PERMANENT_TURTLE_NAME
        
        if(self.name == _PERMANENT_TURTLE_NAME):
            return

        kill_turtle(self.name)

       
    def __del__(self):

        self.clear()
        
        

