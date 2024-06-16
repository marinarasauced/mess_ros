
################################################################################
# MISCELLANEOUS FUNCTIONS
#   Purpose:    define functions relating to the mess_ros cache
#   Funcs:   
#           01. check4path
################################################################################

import os



# Makes directories at path if they do not exist.
#   In:     path
#   Out:    directores made at path
def check4path(path):
    if not os.path.exists(path):
        os.makedirs(path)

