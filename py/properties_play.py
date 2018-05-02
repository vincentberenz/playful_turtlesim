import playful


class time_stamp(playful.Property):

    def fuse(self,value):

        self._value = value


    def similarity(self,value):

        return None
    


class position(playful.Property):

    def fuse(self,value):

        self._value = value


    def similarity(self,value):

        return None

    

class index(playful.Property):

    
    def fuse(self,value):

        self._value=value


    # if two turtles have the same index,
    # then they are the same turtle
    def similarity(self,value):

        if self._value == value:
            return True

        return False


                      

