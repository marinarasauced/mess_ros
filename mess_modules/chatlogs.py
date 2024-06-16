
################################################################################
# CHATLOG FUNCTIONS
#   Purpose:    define functions relating to terminal chatlogs
#   Funcs:   
#            2. 
#            3. 
#            3. 
#            4. 
#            5. 
#            6. 
################################################################################

import rospy



#
#
#
def chatlog_task(msg):
    print(f"\n >>> [Task:  {msg}]")



#
#
#
def chatlog_subtask(msg):
    print(f" >>> [Doing: {msg}]")



#
#
#
def chatlog_done(msg):
    print("\033[92m" + f" >>> [Done:  {msg}]\n" + "\033[0m")



#
#
#
def chatlog_error(msg):
    print("\033[91m" + f" >>> [Error: {msg}]" + "\033[0m")
    #rospy.signal_shutdown("error occurred")