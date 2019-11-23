#! /usr/bin/env python
import roslib
import rospy
from polygon import AreaPolygon
import polygon as poly
from cpr_gps_navigation_msgs.msg import MissionGoal, Waypoint
from cpr_gps_navigation_msgs.srv import AreaCoverage, AreaCoverageResponse
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from std_msgs.msg import String

DEBUG = 1

class CoverageServer(object):

    def __init__(self):
        self.s = rospy.Service('area_coverage', AreaCoverage, self.cover_area)
        rospy.Subscriber("/odometry/filtered_map", Odometry, self.position_cb)

        self.exterior = []
        self.interiors = []
        self.pos = Odometry()
    
    def position_cb(self, odom):
        """Current robot position:
            robot_east: odom.pose.pose.position.x
            robot_north: odom.pose.pose.position.y
        """
        self.pos = odom
            
    def cover_area(self, goal):
        """Service to compute the area coverage path for the robot"""
        rospy.loginfo('Executing area coverage service')

        # Record GPS markers for outer area
        self.exterior = goal.exterior.mission.viapoints
        self.exterior.append(goal.exterior.mission.goalpoint)

        # Record GPS markers for inner obstacle(s)
        n = len(goal.interiors)
        for i in range(n):
            interior = goal.interiors[i].mission.viapoints
            interior.append(goal.interiors[i].mission.goalpoint)
            self.interiors.append(interior)

        # Perform area coverage
        path = self.compute_coverage()

        # Service response
        resp = AreaCoverageResponse()
        resp.coverage = self.send_coverage_path(list(path.coords))
        return resp
    
    def compute_coverage(self):
        # Convert exterior and interior points to tuples
        ext = [(coord.x, coord.y) for coord in self.exterior]

        holes = []
        n = len(self.interiors)
        print("Num holes: {}".format(n))
        for i in range(n):
            hole = [(coord.x, coord.y) for coord in self.interiors[i]]
            holes.append(hole)

        # Determine initial position
        initial_pos = (self.pos.pose.pose.position.x, self.pos.pose.pose.position.y)

        # AreaPolygon(coordinates, initial_pos, ft=1.0, angle=None)
        print(holes)
        polygon = AreaPolygon(ext, initial_pos, holes, ft=0.5)
        ll = polygon.get_area_coverage()
        print(list(ll.coords))

        # Plotting results
        if DEBUG:
            fig = plt.figure(1, dpi=90)
            ax = fig.add_subplot(121)
            poly.plot_coords(ax, ll)
            poly.plot_bounds(ax, ll)
            poly.plot_line(ax, ll)
            plt.plot(*polygon.P.exterior.xy)
            plt.show()

        return ll
    
    def send_coverage_path(self, path):
        """Prepare coverage message to send to mission server"""
        coverage = MissionGoal()
        coverage.mission.header.stamp = rospy.Time.now()
        coverage.mission.goalpoint_theta = 0
        coverage.mission.goalpoint.x, coverage.mission.goalpoint.y = path.pop()
        print("Goal point: %f, %f" % (coverage.mission.goalpoint.x, coverage.mission.goalpoint.y))

        viapoint = Waypoint()
        for via in path:
            x, y = via
            print("Viapoint: %f, %f" % (x, y))
            coverage.mission.viapoints.append(Waypoint(x, y))
        return coverage

        
if __name__ == '__main__':
    try:
        rospy.init_node('areacoverage')
        # AreaCoverage server receives the polygon vertices from the mission_client and computes the coverage
        server = CoverageServer()     
        rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
    except KeyboardInterrupt:
        exit()
    


