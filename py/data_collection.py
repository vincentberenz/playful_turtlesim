
import threading

class TargetNode:

    def  __init__(self,target,node):
        self.target = target
        self.node = node
        self.name = self.get_name(target,node)

    @classmethod
    def get_name(cls,target,node):
        return str(target)+"_"+str(name)
        
class Logger:


    def __init__(self):

        self.activations = []
        self.features = []
        self.current_activation = {}
        self.current_features = []
        self.lock = threading.Lock()

    def set_features(self,features):

        with self.lock:
            self.current_features.append(features)
        
    def set_activation(self,target,node,status):
        
        with self.lock:
            name = TargetNode.get_name(target,node)
            self.current_activation[name]=status
        
    def save(self):

        with self.lock:
            self.activations.append(self.current_activation)
            self.current_activation = {}
            self.features.append(self.current_features)
            self.current_features = []
        
    def set_in_file(self,path):

        with open(path,"w+") as f:
            f.write(repr(self.features))
            f.write("\n")
            f.write(repr(self.activations))
    
    @classmethod
    def read_from_file(cls,path):
        
        with open(path,"r") as f:
            features = eval(f.readline())
            activations = eval(f.readline())

        return features,activations
        
