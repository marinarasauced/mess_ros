#!/usr/bin/env python3

import rospy

import os.path
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../")))

from mess_modules.agents import get_agent_name
from mess_modules.ugv_secondary import UGVSecondary


def main():
    """
    Start the UGVSecondary instance on the agent.

    This node will start the ugv agent's dynamics. It is currently configured for waypoint navigation, but the ugv.transitionUGV() could be replaced with different dynamics if the UGVSecondary class is either modified and expanded or recreated. turtlebot3_bringup MUST finish before this node starts, otherwise the control input publisher may not work as intended.
    """

    rospy.sleep(12)  # wait for turtlebot3_bringup (ikr such a bad way to wait)
    rospy.init_node("ugv_vertex")

    ugv = UGVSecondary(get_agent_name())
    print("ready")  # do not remove
    ugv.transitionUGV()

    rospy.spin()

if __name__=="__main__":
    main()
