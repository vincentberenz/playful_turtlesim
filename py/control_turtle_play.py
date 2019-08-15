import playful
import threading,copy

import rospy
from geometry_msgs.msg import Twist

_VELOCITY_TOPIC = "/turtle1/cmd_vel"
_POSE_TOPIC = "/turtle1/pose"
_SET_PEN_SERVICE = "/turtle1/set_pen"


class rfTurtle:

    def __init__(self,position):
        self.position = position


# transform the position of the target from absolute frame to
# position relative to the robot, and set to memory
# a corresponding instance of rfTurtle (robot frame turtle)
class in_robot_frame(playful.Node):

    def execute(self):

        robot_position = None
        lock = threading.Lock()

        
        def _pose_callback(msg):
            with lock:
                robot_position = [msg.x,msg.y,msg.theta]

        global _POSE_TOPIC
        subscriber = rospy.Subscriber(_POSE_TOPIC,Pose,_pose_callback)

        
        def _get_relative_position(absolute_position,
                                  robot_position):
            theta = robot_position[2]
            v = [a-r for a,r in zip(absolute_position,robot_position)]
            gamma = math.atan2(v[1],v[0])
            beta = gamma-theta
            d = math.sqrt(sum([(a-r)**2 for a,r in zip(absolute_position[:2],robot_position[:2])]))
            position = [d*math.cos(beta),d*math.sin(beta),beta]
            return position

        
        while not self.should_pause():

            target = self.get_target()


            with lock:
                robot_position_ = copy.deepcopy(robot_position)

            
            if target and robot_position_:

                absolute_position = target.position
                position = _get_relative_position(absolute_position,
                                                  robot_position_)
                
                playful.memory.set(rfTurtle(position),id(self))
                
            
            self.spin(20)

                
# swim toward the targeted object.
# assumes the position of the targeted object in
# frame relative to the robot
class swim_toward(playful.Node):

    
    def __init__(self,k_linear=0.2,k_angular=0.4):
        
        self.k_linear = k_linear
        self.k_angular = k_angular

        
    def execute(self):
        
        # for setting the desired velocity of this turtle
        global _VELOCITY_TOPIC
        velocity_publisher = rospy.Publisher(_VELOCITY_TOPIC,
                                             Twist, queue_size=10)

        # ros velocity message
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0

        while not self.should_pause():

            if self.ask_for_resource("legs"):
            
                target = self.get_target()

                if target:

                    # assumed position in relative frame of robot
                    position = target.position

                    if position[0]>0:
                        msg.linear.x = self.k_linear * position[0]
                    else :
                        msg.linear.x = 0
                    msg.angular.z = self.k_angular * math.atan2(position[1],position[0])

                    velocity_publisher.publish(msg)

                self.spin(20)

        

class set_color(playful.Node):

    def __init__(self,r=0,g=0,b=0,width=5):
        self.r = r
        self.g = g
        self.b = b
        self.width = width

    def execute(self):

        def _set_pen(self,r,g,b,width):

            global _SET_PEN_SERVICE
            rospy.wait_for_service(self.name+"/"+_SET_PEN_SERVICE)
            service = rospy.ServiceProxy(self.name+"/"+_SET_PEN_SERVICE, SetPen)
            response = service(r, g, b, width, 0)

        while not self.should_pause():

            if self.ask_for_resource("pen"):

                self._set_pen(self.r,
                              self.g,
                              self.b,
                              self.width)

                self.spin(20)
