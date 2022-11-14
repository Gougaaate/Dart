import fsm
import time
import sys
import dummyrob

# global variables
f = fsm.fsm();  # finite state machine

time_start = time.time()
time_begin = 5.0 # wait 5 s before turning
turn_count = 0 # count half turns

# create a robot
myBot = dummyrob.MyDummyRob()

# functions (actions of the fsm)
def doWait():
    global time_start,time_begin
    print (">>>>>> action")
    myBot.doNothing(1)
    if time.time() - time_start > time_begin: # auto start in 5 seconds
        event="go"
    else:
        event="stop"
    return event

def doTurn():
    global turn_count
    turn_count = turn_count+1
    print (">>>>>> action")
    myBot.halfTurn(3)
    if turn_count == 2: # full turn
        event = "stop"
    else:
        event = "go"
    return event
   
def doStop():
    print (">>>>>> action : stop all")
    event=None
    return event

if __name__== "__main__":
    
    # load fsm definition from a text file
    f.load_fsm_from_file ("fsm_full_turn.txt")
 
    # fsm loop
    run = True   
    while (run):
        funct = f.run () # function to be executed in the new state
        if f.curState != f.endState:
            newEvent = funct() # new event when state action is finished
            print ("New Event : ",newEvent)
            if newEvent is None:
                break
            else:
                f.set_event(newEvent) # set new event for next transition
        else:
            funct()
            run = False
            
    print ("End of the programm")



