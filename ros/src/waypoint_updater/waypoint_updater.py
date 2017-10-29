#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Header

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        # Waypoint publisher
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)


        # TODO: Add other member variables you need below
        self.waypoints = None
        self.current_position = None

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        rospy.loginfo('current_pose callback received, position: %f', msg.pose.position.x)
        self.current_position = msg.pose.position
        self.publish_waypoints()
        pass

    def waypoints_cb(self, msg):
        # TODO: Implement
        rospy.loginfo('base_waypoints callback received.')

        if self.waypoints is None:
            self.waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def publish_waypoints(self):
        # TODO: Implement waypoints publishing
        # NOTE: not sure yet, but I think we need to publish waypoints only on  current_pose event, otherwise we don't know
        # position
        if self.current_position is None:
            return

        next_waypoint_index = self.next_waypoint(self.current_position)
        rospy.loginfo('next_waypoint_index = %d', next_waypoint_index)

        next_waypoints = self.waypoints[next_waypoint_index:min(next_waypoint_index+LOOKAHEAD_WPS, len(self.waypoints))]
        rospy.loginfo('num next_waypoints %d', len(next_waypoints))


        # self.max_speed = rospy.get_param('~/waypoint_loader/velocity')


        # create and publish ros message
        h = Header()
        h.stamp = rospy.Time.now()
        msg = Lane()
        msg.header = h
        msg.waypoints = next_waypoints
        self.final_waypoints_pub.publish(msg)

        rospy.loginfo('published final_waypoints')



    def next_waypoint(self, position):
        # Find closest waypoint
        waypoint_index = self.get_closest_waypoint(position)
        waypoint = self.waypoints[waypoint_index]

        # Calculate heading
        heading = math.atan2(waypoint.pose.pose.position.y - position.y, waypoint.pose.pose.position.x - position.x)
        yaw = self.get_waypoint_yaw(waypoint)
        angle = abs(yaw - heading)

        # If waypoint have same heading then we found it else return next waypoint index
        if angle > math.pi/4:
            waypoint_index += 1

        return waypoint_index


    def get_closest_waypoint(self, position):
        min_distance = None
        waypoint_index = None

        for i in range(len(self.waypoints)):
            distance = pow(self.waypoints[i].pose.pose.position.x - position.x, 2) + pow(self.waypoints[i].pose.pose.position.y - position.y, 2)
            if min_distance is None or distance < min_distance:
                min_distance = distance
                waypoint_index = i

        return waypoint_index


    def get_waypoint_yaw(self, waypoint):
        orientation = waypoint.pose.pose.orientation

        euler = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        # roll = euler[0]
        # pitch = euler[1]
        # yaw = euler[2]

        return euler[2]

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
