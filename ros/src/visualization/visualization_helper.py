#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLightArray, TrafficLight
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float32
from dbw_mkz_msgs.msg import BrakeCmd, ThrottleCmd, SteeringCmd

class VisualizationHelper(object):
    def __init__(self):
        rospy.init_node("visualization_helper")
        self.rate = rospy.Rate(5)
        self.loop()


    def publish(self):
        while not rospy.is_shutdown():

            self.rate.sleep()


if __name__ == '__main__':
    try:
        VisualizationHelper()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start visualization helper node.')
