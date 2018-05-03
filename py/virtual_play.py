import playful,time

from playful_turtlesim.py.turtlesim_wrapper import Turtlesim


class set_target(playful.Node):

    
    def __init__(self,position=[0,0,0]):

        self._position = position


    def execute(self):

        spawned = False
        ros_turtle = None
        
        while not self.should_pause():

            if not spawned:
                #ros_turtle = Turtlesim(self._position[0],self._position[1],0)
                spawned = True
                
            target = playful.create("target",
                                    position=self._position,
                                    time_stamp=time.time(),
                                    index=0)
                    
            playful.memory.fuse(target)

            self.spin(15)

        if ros_turtle:
            ros_turtle.clear()
