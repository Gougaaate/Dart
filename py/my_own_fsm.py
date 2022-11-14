import fsm
import time
import xXDartSasukeXx

# global variables
FSM = fsm.fsm()  # finite state machine

# create a robot (to be replaced by dartv2)
myBot = xXDartSasukeXx.xXDartSasukeXx()

time_start = time.time()
time_begin = 5.0 # wait 5 s before turning

# functions (actions of the fsm)
def doWait():
	global time_start,time_begin
	if time.time() - time_start > time_begin: # auto start in 5 seconds
		event="go"
		print ("It's time to go forward during 5s")
		time_start = time.time()
	else:
		event="wait"
	return event

def goForward():
	global time_start
	myBot.set_motor_cmd(100,100)
	if time.time()-time_start<5:
		event = "go_forward"
	else:
		print("I will go back now")
		event = "change_side"
		time_start = time.time()
	return event

def goBackward(t0=time.time()):
	myBot.set_motor_cmd(-100,-100)
	if time.time()-time_start<5:
		event = "go_backward"
	else:
		print("I will stop soon")
		event = "stop"
	return event
   
def doStop():
	myBot.stop()
	print ("I stop myself!")
	event=None
	return event

if __name__== "__main__":
	
	# define the states
	FSM.add_state ("Idle")            # Ne rien faire
	FSM.add_state ("Go_Forward")
	FSM.add_state ("Go_Backward")
	FSM.add_state ("End")
   
	# Transition entre les états

	FSM.add_transition ("Idle", "Idle", "wait", doWait)
	FSM.add_transition ("Idle", "Go_Forward", "go", goForward)
	FSM.add_transition ("Go_Forward", "Go_forward", "go", goForward)
	FSM.add_transition ("Go_Forward", "Go_Backward", "change_side", goBackward)
	FSM.add_transition ("Go_Backward", "Go_Forward", "change_side", goForward)
	FSM.add_transition ("Go_Backward", "Go_Backward", "go_backward", goBackward)
	FSM.add_transition ("Go_Backward", "End", "stop", doStop)

	# Définition des événements

	FSM.add_event ("wait")
	FSM.add_event ("go")
	FSM.add_event ("go_forward")
	FSM.add_event ("change_side")
	FSM.add_event ("go_backward")
	FSM.add_event ("stop")

	FSM.set_state ("Idle")
	FSM.set_event ("wait")
	FSM.set_end_state ("End")

 
	run = True
	while (run):
		funct = FSM.run() 								 # function to be executed in the new state
		if FSM.curState != FSM.endState:
			newEvent = funct() 							 # new event when state action is finished
			print ("New Event : ",newEvent)
			FSM.set_event(newEvent)						 # set new event for next transition
		else:
			funct()
			run = False
			
	print ("End of the programm")

