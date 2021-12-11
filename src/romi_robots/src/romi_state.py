from enum import Enum

# The FSM model of Romi can be described using two separate state machines
# working in sync, with each state in the physical romi interacting with a 
# state machine in the digial romi.

# Physical Romi States:       Digital Romi States:
#       Scanning       ------>       Idling
#         Send         ------>       Receive
#        Receive       ------>        Send
#         Drive        ------>       Idling
#         Close        ------>        Close

class State(Enum):
    Receive = 0
    Send = 1
    Close = 2