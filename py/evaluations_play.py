import playful,time,math


# distance is already defined in playful_tutorial/py/tutorial_play.py
def distance(scheme_id=None):

    target_position = playful.memory.get_property_value("position",scheme_id=scheme_id)

    if not target_position :
        return float("+inf")

    robot = playful.get_global("ROBOT")
    robot_position = robot.get_position()

    if not robot_position :
        return float("+inf")

    d = math.sqrt(sum([(r-t)**2 for r,t in
                       zip(robot_position[:2],target_position[:2])]))
    
    return d


def inv_distance(scheme_id=None,base_score=0):

    d = distance(scheme_id=scheme_id)
    return base_score + 1.0/d


def abs_angle(scheme_id=None):

    target_position = playful.memory.get_property_value("position",scheme_id=scheme_id)

    if not target_position :
        return float("+inf")

    robot = playful.get_global("ROBOT")

    relative_position = robot.in_robot_frame(target_position)
    
    if not relative_position :
        return float("+inf")

    return abs( relative_position[2] )


def low_battery(battery_threshold=20):

    robot = playful.get_global("ROBOT")
    charge = robot.get_battery_level()

    if charge < battery_threshold:
        return True

    return False


def high_battery(battery_threshold=80):

    robot = playful.get_global("ROBOT")
    charge = robot.get_battery_level()

    if charge > battery_threshold:
        return True

    return False


# support class for in_turn evaluation (below)
class _In_turn:
    
    scheme_ids = []
    last_time_started = None
    current_index = None

    @classmethod
    def is_active(cls,scheme_id,duration):
        
        if scheme_id not in cls.scheme_ids:
            cls.scheme_ids.append(scheme_id)

        t = time.time()
            
        if cls.current_index is None:
            cls.current_index = 0
            cls.last_time_started = t

        if t-cls.last_time_started > duration:
            cls.current_index+=1
            if cls.current_index >= len(cls.scheme_ids):
                cls.current_index = 0
            cls.last_time_started = t

        if scheme_id == cls.scheme_ids[cls.current_index]:
            return True

        return False
            

def in_turn(scheme_id=None,duration=8):

    return _In_turn.is_active(scheme_id,duration)
