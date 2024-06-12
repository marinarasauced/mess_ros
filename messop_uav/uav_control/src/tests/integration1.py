#!/usr/bin/env python

import rospy

import os.path
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.abspath(__file__), "../../../../")))

from uav_modules.uav_class import UAV

##################################################################################

#
def run_integration1(uav):
    test = 1

##################################################################################

def main():

    # Initialize node:
    rospy.init_node("uav_integration1")
    rospy.sleep(12)  # wait for mavros

    # Create UAV object:
    uav = UAV("hawk2", 0.0, 0.0, 0.0, 1.0)

    # Run integration test:
    run_integration1(uav)

if __name__=="__main__":
    main()