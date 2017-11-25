#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''


class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # min_speed = 0.0

        self.rate = 10  # in Hz, don't forget to change to 50 later

        # TODO: Create `TwistController` object
        self.controller = Controller(
            vehicle_mass=vehicle_mass,
            fuel_capacity=fuel_capacity,
            brake_deadband=brake_deadband,
            decel_limit=decel_limit,
            accel_limit=accel_limit,
            wheel_radius=wheel_radius,
            wheel_base=wheel_base,
            steer_ratio=steer_ratio,
            max_lat_accel=max_lat_accel,
            max_steer_angle=max_steer_angle
        )

        # TODO: Subscribe to all the topics you need to
        self.sub_twist = rospy.Subscriber('/twist_cmd', TwistStamped, self.cb_twistmsg)
        self.sub_dbw_status = rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.cb_dbw_status)
        self.sub_curr_vel = rospy.Subscriber('/current_velocity', TwistStamped, self.cb_currvel)

        # Current state variables
        self.dbw_is_enabled = True

        self.current_linear_velocity = None
        self.current_angular_velocity = None
        self.proposed_linear_velocity = None
        self.proposed_angular_velocity = None
        self.previous_time = rospy.get_rostime()

        self.loop()

    def loop(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            # getting time step for PID controllers
            this_time = rospy.get_rostime()
            time_step = (this_time - self.previous_time).secs + 1e-9 * (this_time - self.previous_time).nsecs
            self.previous_time = this_time

            if self.current_linear_velocity is None or self.proposed_linear_velocity is None:
                continue

            if self.dbw_is_enabled:
                throttle, brake, steering = self.controller.control(self.proposed_linear_velocity,
                                                                    self.proposed_angular_velocity,
                                                                    self.current_linear_velocity,
                                                                    self.current_angular_velocity,
                                                                    time_step)
                self.publish(throttle, brake, steering)
            else:
                self.controller.reset();
            rate.sleep()

    def publish(self, throttle, brake, steer):
        rospy.loginfo('publishing dbw controller output : throttle %f, brake %f, steer %f', throttle, brake, steer)
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def cb_twistmsg(self, msg):
        self.proposed_linear_velocity = msg.twist.linear.x
        self.proposed_angular_velocity = msg.twist.angular.z

    def cb_dbw_status(self, msg):
        self.dbw_is_enabled = msg

    def cb_currvel(self, msg):
        self.current_linear_velocity = msg.twist.linear.x


if __name__ == '__main__':
    DBWNode()
