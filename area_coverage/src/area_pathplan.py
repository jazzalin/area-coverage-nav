#! /usr/bin/env python
from __future__ import print_function
import actionlib
from cpr_gps_navigation_msgs.msg import MissionAction

class PathplanClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient('missionplan', MissionAction)
        self.client.wait_for_server()

    def sendMission(self, path):
        """Send path comprised of waypoints to the MissionServer"""
        self.client.send_goal(path)
        self.client.wait_for_result()
        return self.client.get_result()
