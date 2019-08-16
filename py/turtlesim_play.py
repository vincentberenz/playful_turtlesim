import playful
import threading,copy,random,time,math

import rospy
from geometry_msgs.msg import Twist

_VELOCITY_TOPIC = "/turtle1/cmd_vel"
_POSE_TOPIC = "/turtle1/pose"
_SET_PEN_SERVICE = "/turtle1/set_pen"
_TELEPORT_SERVICE = "teleport_absolute"
_SPAWN_SERVICE = "spawn"
_KILL_SERVICE = "kill"


from turtlesim.srv import Spawn
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import Kill
from turtlesim.srv import SetPen
from turtlesim.msg import Pose
from std_srvs.srv import Empty

# a turtle with position expressed in the
# absolute frame
class Turtle(object):
    
    __slots__=["position"]
    
    def __init__(self,position):
        self.position = position


# a turtle with position expressed in
# the frame relative to the controlled turtle
class rfTurtle:

    def __init__(self,position):
        self.position = position


# the home of the controlled robot
class Home:

    def __init__(self,position):
        self.position = position

        
# the home of the controlled robot
# in frame relative to controlled turtle
class rfHome:

    def __init__(self,position):
        self.position = position

        


# help function to get controlled turtle position
class _Robot_position:

    _subscriber = None
    _lock = threading.Lock()
    _robot_position = None
    
    @classmethod
    def init(cls):
        if cls._subscriber is not None:
            return
        cls._robot_position = None
        def _pose_callback(msg):
            with cls._lock:
                cls._robot_position = [msg.x,msg.y,msg.theta]
        global _POSE_TOPIC
        cls._subscriber = rospy.Subscriber(_POSE_TOPIC,Pose,_pose_callback)

    @classmethod
    def get(cls):
        cls.init()
        with cls._lock:
            return copy.deepcopy(cls._robot_position)
        

        
       
# transform the position of the target from absolute frame to
# position relative to the robot, and set to memory
# a corresponding instance of rfTurtle (robot frame turtle)
class in_robot_frame(playful.Node):

    def __init__(self,out=None):
        self.out = out
    
    def execute(self):

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
            robot_position = _Robot_position.get()
            
            if target and robot_position:

                absolute_position = target.position
                position = _get_relative_position(absolute_position,
                                                  robot_position)
                
                playful.Memory.set(globals()[self.out](position),id(self))
                
            
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

            else :
                
                self.release_all_resources()    
                    
            self.spin(20)

        
# set the color and width of the controlled robot trail
class set_color(playful.Node):

    def __init__(self,r=0,g=0,b=0,width=5):
        self.r = r
        self.g = g
        self.b = b
        self.width = width

    def execute(self):

        def _set_pen(r,g,b,width):

            global _SET_PEN_SERVICE
            rospy.wait_for_service(self.name+"/"+_SET_PEN_SERVICE)
            service = rospy.ServiceProxy(self.name+"/"+_SET_PEN_SERVICE, SetPen)
            response = service(r, g, b, width, 0)

        while not self.should_pause():

            if self.ask_for_resource("pen"):

                _set_pen(self.r,
                         self.g,
                         self.b,
                         self.width)

            else :

                self.release_all_resources()
                
            self.spin(20)

                
# set the position of the controlled turtle's home
class set_home(playful.Node):

    def __init__(self,x=0,y=0):
        self.x = x
        self.y = y

    def execute(self):

        while not self.should_pause():

            playful.Memory.set(Home([self.x,self.y]))
            self.spin(5)





def distance_to_home(target=None):

    if not target:
        return float("+inf")

    home = playful.Memory.get("Home")
    if not home:
        return float("+inf")

    d = math.sqrt(sum([(p-h)**2 for p,h
                       in zip(target.position,home.position)]))

    return d



def inv_distance_to_home(target=None,score_range=[0,1]):

    d = distance_to_home(target=target)
    if d==float("+inf"):
        return score_range[1]
    range_diff = score_range[1]-score_range[0]
    r = score_range[0] + range_diff * ( (d+1) / (d+2))
    return r


# check if the target is close to the
# controlled robot's home
def close_to_home(target=None,threshold=5.0):

    d = distance_to_home(target=target)
    return d<threshold




def far_from_robot(target=None,
                   threshold=1):

    if not target:
        return False
    
    robot_position = _Robot_position.get()    

    if robot_position is not None:
        d = math.sqrt(sum([(p-r)**2 for p,r
                       in zip(target.position,robot_position)]))

        return d>threshold

    return False







# set a Turtle in playful memory and in
# turtlesim simulator
# This turtle moves randomly 
class set_turtle(playful.Node):

    
    def __init__(self,
                 min_x=1,max_x=9,
                 min_y=1,max_y=9,
                 min_speed=0.4,max_speed=0.6):

        self._min = [min_x,min_y]
        self._max = [max_x,max_y]
        self._speed_limit = [min_speed,max_speed]
        self._position,_ = self._get_next_position()
        self._last_time_update = time.time()
        self._set_next_move()

        
    def execute(self):

        global _TELEPORT_SERVICE

        first_time = True
        
        while not self.should_pause():

            # spawning the turtle
            if first_time:
                self._name = self._spawn(self._position[0],
                                         self._position[1],
                                         0)
                first_time = False
                
            # updating turtle position
            # (set random positions to go to)
            self._iterate()

            # setting it in ROS
            rospy.wait_for_service(self._name+"/"+_TELEPORT_SERVICE)
            service = rospy.ServiceProxy(self._name+"/"+_TELEPORT_SERVICE, TeleportAbsolute)
            service(self._position[0],
                    self._position[1],
                    0)

            # setting it in playful
            turtle = Turtle(self._position)
            playful.Memory.set(turtle,self._name)

        # deleting the turtle on exit
        self.kill_turtle(self._name)
            

    # generate randomly next position the turtle should go to
    def _get_next_position(self):

        x = random.uniform(self._min[0],self._max[0])
        y = random.uniform(self._min[1],self._max[1])

        speed = random.uniform(self._speed_limit[0],
                           self._speed_limit[1])

        return [x,y],speed

    
    def _distance(self,p1,p2):
        return math.sqrt(sum([(a-b)**2 for a,b in zip(p1,p2)]))
    

    # has the robot move step by step to currrent target position
    def _set_next_move(self):

        position,self._speed = self._get_next_position()
        self._target_distance = self._distance(position,self._position)
        vector = [p2-p1 for p1,p2 in zip(self._position,position)]
        norm = math.sqrt(sum([v**2 for v in vector]))
        self._vector = [v/norm for v in vector]
        self._started_position = [p for p in self._position]
        

    # check if turtle arrived to target position
    def _arrived(self):

        d = self._distance(self._position,self._started_position)

        if d > self._target_distance:
            return True

        return False

    
    # manages next position the robot should go to, 
    # by directing turtle to target position,
    # generating a new target position when suitable
    def _iterate(self):

        if (self._arrived()):
            self._set_next_move()
        
        t = time.time()
        time_diff = t-self._last_time_update
        traveled = self._speed * time_diff
        self._last_time_update = t
        
        self._position = [p + v*traveled for p,v in zip(self._position,self._vector)]

        
    # generate the new turtle in ROS
    def _spawn(self,x,y,theta):

        global _SPAWN_SERVICE
        
        rospy.wait_for_service(_SPAWN_SERVICE)
        service = rospy.ServiceProxy(_SPAWN_SERVICE, Spawn)
        response = service(x, y, theta, None)

        return response.name

    
    # delete the turtle in ROS
    @staticmethod
    def kill_turtle(turtle_name):

        try:
            rospy.init_node('kill_turtle_'+turtle_name, anonymous=True)
        except :
            pass

        global _KILL_SERVICE

        rospy.wait_for_service(_KILL_SERVICE)
        service = rospy.ServiceProxy(_KILL_SERVICE, Kill)
        service(turtle_name)

        
    def __del__(self):
        self.kill_turtle(self._name)
