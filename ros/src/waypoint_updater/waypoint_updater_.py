#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Header
from std_msgs.msg import Int32
from velocity_updater import *

import math

from dynamic_reconfigure.server import Server
from waypoint_updater.cfg import JMTParamsConfig


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

LOOKAHEAD_WPS = 40 # Number of waypoints we will publish. You can change this number
STOPPING_DIST = 1.
MAX_DECEL = 1.

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # Waypoint publisher
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rospy.Subscriber('/current_velocity', TwistStamped, self.vel_cb)

        # TODO: Add other member variables you need below
        self.max_speed = self.kmph2mps(rospy.get_param('~/waypoint_loader/velocity'))


        self.mode = rospy.get_param('~mode', 'linear')

        self.waypoints = None
        self.current_position = None
        self.current_velocity = None
        self.velocity_updater = VelocityUpdater(self.max_speed, rospy)
        self.traffic_light = None
        self.hasReceivedTrafficMsg = False

        self.srv = Server(JMTParamsConfig, self.config_callback)

        self.loop()
        #rospy.spin()


    def loop(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            self.publish_waypoints()
            rate.sleep()

    def config_callback(self, config, level):
        rospy.loginfo("Updating JMT weights")
        self.velocity_updater.update_weights(config)
        return config

    def pose_cb(self, msg):
        # TODO: Implement
        #rospy.loginfo('current_pose callback received, position: %f', msg.pose.position.x)
        self.current_position = msg.pose.position


    def vel_cb(self, msg):
        #rospy.loginfo('current_velocity callback received, velocity: %f', msg.twist.linear.x)
        self.current_velocity = msg.twist.linear.x

    def waypoints_cb(self, msg):
        # TODO: Implement
        #rospy.loginfo('base_waypoints callback received.')

        if self.waypoints is None:
            self.waypoints = msg.waypoints

    def traffic_cb(self, msg):

        if not self.hasReceivedTrafficMsg:
            self.hasReceivedTrafficMsg = True

        if msg.data < 0:
            self.traffic_light = None
        else:
            self.traffic_light = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def publish_waypoints(self):
        if self.current_position is None or self.waypoints is None or not self.hasReceivedTrafficMsg:
            return

        next_waypoint_index = self.next_waypoint(self.current_position)
        # rospy.loginfo('next_waypoint_index = %d', next_waypoint_index)

        last_waypoint_index = min(next_waypoint_index+LOOKAHEAD_WPS, len(self.waypoints))
        next_waypoints = self.waypoints[next_waypoint_index:last_waypoint_index]

        if self.mode == 'linear':
            if self.traffic_light is not None:
                tf_relative_position = self.traffic_light - next_waypoint_index
                if tf_relative_position <= LOOKAHEAD_WPS and tf_relative_position > 0:
                    rospy.loginfo('slowing down')
                    slow_down_waypoints = self.waypoints[next_waypoint_index:self.traffic_light + 1]
                    self.decelerate(slow_down_waypoints)
            else:
                for wpidx in range(next_waypoint_index, last_waypoint_index):
                    self.set_waypoint_velocity(self.waypoints, wpidx, self.max_speed)
        elif self.mode == 'JMT':
            if self.traffic_light is None:
                self.velocity_updater.update(next_waypoints, next_waypoint_index, self.current_velocity, None)
            else:
                tf_relative_position = self.traffic_light - next_waypoint_index
                if tf_relative_position > LOOKAHEAD_WPS or tf_relative_position < 0:
                    tf_relative_position = None
                self.velocity_updater.update(next_waypoints, next_waypoint_index, self.current_velocity, tf_relative_position)


        i = 0
        speeds = ''
        while i < 10: #len(next_waypoints):
            speeds = speeds + "{0:.2f} ".format(self.get_waypoint_velocity(next_waypoints[i]))
            i = i + 1

        rospy.loginfo('Speeds ' + speeds)

        # create and publish ros message
        h = Header()
        h.stamp = rospy.Time.now()
        msg = Lane()
        msg.header = h
        msg.waypoints = next_waypoints
        self.final_waypoints_pub.publish(msg)

       # rospy.loginfo('published final_waypoints')



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

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)

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

    def distance_positions(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def decelerate(self, waypoints):
        last = waypoints[-1]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:-1][::-1]:
            dist = max( self.distance_positions(wp.pose.pose.position, last.pose.pose.position) - STOPPING_DIST, 0)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
        return waypoints


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
