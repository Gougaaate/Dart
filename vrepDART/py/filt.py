import numpy as np
import time

class LowPassFilter ():
    
    def __init__(self): 
        self.a = 1.0  # here you define some useful variables 
        pass
    
    def simpleLowPassFilter(self,v):
        """
        Add your filtering code here, now the filter does nothing
        """
        vf = self.a*v
        return vf
