import playful,math,time


class swim_forward(playful.Node):


    def __init__(self,kp=0.16):

        self._kp = kp


    def execute(self):

        robot = playful.get_global("ROBOT")
        
        while not self.should_pause():

            if self.ask_for_resource("legs"):
            
                # in absolute frame
                target = playful.memory.get_property_value("position",scheme_id=self.get_scheme_id())

                # in robot frame
                target_ = robot.in_robot_frame(target)
                
                if target:

                    control = self._kp * target_[0]
                    robot.set_velocity(control,None)

            else :

                self.release_all_resources()

            self.spin(15)



class turn(playful.Node):


    def __init__(self,kp=0.8):

        self._kp = kp


    def execute(self):

        robot = playful.get_global("ROBOT")
        
        while not self.should_pause():

            if self.ask_for_resource("tail"):
            
                # in absolute frame
                target = playful.memory.get_property_value("position",scheme_id=self.get_scheme_id())

                # in robot frame
                target_ = robot.in_robot_frame(target)

                if target:

                    angle = math.atan2(target_[1],target_[0])
                    control = self._kp * angle
                    robot.set_velocity(None,control)

            else :

                self.release_all_resources()

            self.spin(15)

            
            
# stopping

class stop(playful.Node):

    def execute(self):

        robot = playful.get_global("ROBOT")
        
        while not self.should_pause():

            if self.ask_for_resource("legs"):

                robot.set_velocity(0,None)

            else:

                self.release_resource("legs")

            if self.ask_for_resource("tail"):

                robot.set_velocity(None,0)

            else:

                self.release_resource("tail")
                
            self.spin(15)

            
# direct control of velocity, for testing

class set_velocity(playful.Node):


    def __init__(self,x=0,theta=0):

        self._x = x
        self._theta = theta


    def execute(self):

        robot = playful.get_global("ROBOT")

        while not self.should_pause():

            robot.set_velocity(self._x,self._theta)

            self.spin(15)



class set_pen_color(playful.Node):

    def __init__(self,color=[255,255,255],width=5):

        self._color = color
        self._width = 5

    def execute(self):

        done = False
        robot = playful.get_global("ROBOT")

        while not self.should_pause():

            if self.ask_for_resource("brush"):

                if not done:

                    R,G,B = self._color
                    robot.set_pen(R,G,B,self._width)
                    done = True

            else:

                done = False
                self.release_all_resources()

            self.spin(10)

                    
                    
