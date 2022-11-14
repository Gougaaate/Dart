import sys

class fsm:
    def __init__(self):
        self.transitions = {}
        self.states = []
        self.events = []
        self.curState = None
        self.curEvent = None
        self.prevState = None
        self.endState = None

    def str2fun(self,astr):
        module, _, function = astr.rpartition('.')
        if module:
            __import__(module)
            mod = sys.modules[module]
        else:
            mod = sys.modules['__main__']  # or whatever's the "default module"
        return getattr(mod, function)

    def load_fsm_from_file (self,file_fsm):
        ffsm = open(file_fsm)
        mode = None
        for l in ffsm.readlines():
            l = l[0:-1] # remove cr
            if l.startswith("----- States"):
                mode = "st"
            elif l.startswith("----- Events"):
                mode = "ev"
            elif l.startswith("----- Transitions"):
                mode = "tr"
            elif l.startswith("---- Start State"):
                mode = "ss"
            elif l.startswith("---- Start Event"):
                mode = "se"
            elif l.startswith("---- End State"):
                mode = "es"
            else:
                if mode == "ss":
                    self.curState = l
                if mode == "es":
                    self.endState = l
                if mode == "se":
                    self.curEvent = l
                elif mode == "tr":
                    sl = l.split(" ")
                    func = self.str2fun(sl[3])
                    self.add_transition (sl[0],sl[1],sl[2],func)
                elif mode == "ev":
                    self.add_event (l)
                elif mode == "st":
                    self.add_state (l)
        ffsm.close()

    def add_transition(self, state1, state2, event, funct):
        key = state1+'.'+event
        self.transitions [key] = (state2, funct)

    def add_state(self, state):
        self.states.append(state)

    def add_event(self, event):
        self.events.append(event)

    def set_state(self, state):
        self.curState = state

    def set_end_state(self, state):
        self.endState = state

    def set_event(self, event):
        self.curEvent = event

    def run(self):
        event = self.curEvent
        state = self.curState
        key = state+'.'+event
        self.prevState = state
        self.curState = self.transitions [key][0]
        if self.prevState != self.curState:
            st = "Transition - Old State : "+state+"; Event : "+event+"; New state : "+self.curState
            st = st+"; Action : "+self.transitions [key][1].__name__+"()"
            print (st)
        return self.transitions [key][1]
