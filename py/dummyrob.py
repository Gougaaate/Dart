import time

class MyDummyRob():

    def goLine (self,duration):
        print ("Linear move for",duration,"s")
        time.sleep(duration)

    def halfTurn (self,duration):
        print ("Half turn in",duration,"s")
        time.sleep(duration)

    def doNothing (self,duration):
        print ("Do nothing for",duration,"s")
        time.sleep(duration)
        
        
    
