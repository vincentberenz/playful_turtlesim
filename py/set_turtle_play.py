import playful,rospy
from turtlesim.srv import Spawn
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import Kill
from turtlesim.srv import SetPen
from turtlesim.msg import Pose
from std_srvs.srv import Empty



# ROS services for communicating with turtles

_TELEPORT_SERVICE = "teleport_absolute"
_SPAWN_SERVICE = "spawn"
_KILL_SERVICE = "kill"


# object that will be used in playful script for targeting
# e.g.
# targeting Turtle: turn_toward

class Turtle(object):
    
    __slots__=["position"]
    
    def __init__(self,position):
        self.position = position


# set a Turtle in playful memory and in
# turtlesim simulator
class set_turtle(playful.Node):

    
    def __init__(self):

        self._min = [min_x,min_y]
        self._max = [max_x,max_y]
        self._speed_limit = [min_speed,max_speed]
        self._position,_ = self._get_next_position()
        self._last_time_update = time.time()
        self._set_next_move()

        
    def __execute__(self):

        global _TELEPORT_SERVICE

        first_time = True
        
        while not self.should_pause():

            # spawning the turtle
            if first_time:
                self._name = self._spawn(self.x,
                                         self.y,
                                         self.theta)
                first_time = False
                
            # updating turtle position
            # (set random positions to go to)
            self._iterate()

            # setting it in ROS
            rospy.wait_for_service(self._name+"/"+_TELEPORT_SERVICE)
            service = rospy.ServiceProxy(self._name+"/"+_TELEPORT_SERVICE, TeleportAbsolute)
            service(x, y, theta)

            # setting it in playful
            turtle = Turtle(self._position)
            playful.memory.set(turtle,self._name)

        # deleting the turtle on exit
        self.kill_turtle(self._name):
            

    # generate randomly next position the turtle should go to
    def _get_next_position(self):

        x = random.uniform(self._min[0],self._max[0])
        y = random.uniform(self._min[1],self._max[1])

        speed = random.uniform(self._speed_limit[0],
                           self._speed_limit[1])

        return [x,y],speed


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

