import playful,math,time


# example of a trivial filter.
# pushes an updated target which position
# is the estimated position in x seconds

class _Predictor:

    def __init__(self,delay):

        self._previous_position = None
        self._previous_time = None
        self._delay = delay
        
    def predict(self,position):

        if position is None:
            return None

        t = time.time()
        
        if self._previous_position is None:
            self._previous_position = [p for p in position]

        if self._previous_time is None:
            self._previous_time = t

        time_diff = t - self._previous_time

        if time_diff == 0:
            self._previous_position = [p for p in position]
            self._previous_time = t
            return [p for p in position]

        velocity = [(current-previous)/time_diff
                    for previous,current in zip(self._previous_position,position)]


        estimated = [p + v*self._delay for p,v in zip(position,velocity)]
        
        self._previous_position = [p for p in position]
        self._previous_time = t

        return estimated

    
class predict_position(playful.Node):

    
    def __init__(self,delay=1,push_type="predicted_target"):

        self.delay = delay
        self.push_type = push_type


    def execute(self):

        predictor = _Predictor(self.delay)
        
        while not self.should_pause():

            scheme_id = self.get_scheme_id()
            position = playful.memory.get_property_value("position",scheme_id=scheme_id)
            index = playful.memory.get_property_value("index",scheme_id=scheme_id)

            if position and (index is not None):
           
                estimated_position = predictor.predict(position)

                estimated_target = playful.create(self.push_type,
                                        position=estimated_position,
                                        time_stamp=time.time(),
                                        index=index)
                    
                playful.memory.fuse(estimated_target)

            self.spin(15)
