import numpy as np
import math
from trajectory import *

MAX_JERK = 10 # m/s/s/s
MAX_ACCEL = 10 # m/s/s

#MAX_T = 10

T_STEPS = 10

class VelocityUpdater(object):
    def __init__(self, max_velocity, rospy, min_velocity = 0, velocity_eps = 0.5):
        self.max_velocity = max_velocity
        self.min_velocity = min_velocity
        self.rospy = rospy
        pass

    def update(self, waypoints, velocity, traffic_light):
        start = [0, velocity, 0]
        distance = None
        end_v = None
        end_ind = None

        if traffic_light is None:
            #we need to reach max speed
            end_ind = len(waypoints) - 1
            end_v = self.max_velocity
        else:
            #we need to slowdown
            end_ind = traffic_light
            end_v = 0

        distance = self.distance(waypoints, 0, end_ind)
        max_t = 10
        if abs(velocity+end_v)/2 > 0.1:
            max_t = distance / (abs(velocity+end_v)/8) #self.max_velocity
        trajectories = []
        for i in range(T_STEPS):
            trajectory = Trajectory(start, [distance, end_v, 0], (i + 1) * max_t / T_STEPS, self.max_velocity)
            trajectories.append(trajectory)

        #find best trajectory
        trajectory = min(trajectories, key=lambda tr: tr.cost())

        #trajectory = None
        #set waypoint speeds
        #we can evaluate s, d_s,dd_s at time T, but we need to know d_s at s
        t = 0
        dt = 0.05
        positions = []
        velocities = []
        while t < trajectory.time:
            positions.append(trajectory.position(t))
            velocities.append(trajectory.velocity(t))
            t += dt

        ind = 0
        for wp_ind in range(end_ind+1):
            dst = self.distance(waypoints, 0, wp_ind)
            while ind < len(positions)-1 and positions[ind] < dst:
                ind += 1

            self.set_waypoint_velocity(waypoints, wp_ind, velocities[ind])

        if end_ind < len(waypoints) and not traffic_light is None:
            for wp_ind in range(end_ind+1, len(waypoints)):
                self.set_waypoint_velocity(waypoints, wp_ind, 0)

        print "velocity start={} end={} time={} cost={}".format(trajectory.start[1], trajectory.end[1], trajectory.time, trajectory.cost())
        print "velocity {}+{}*t+{}*t^2+{}*t^3+{}*t^4+{}*t^5".format(trajectory.a[0], trajectory.a[1], trajectory.a[2], trajectory.a[3], trajectory.a[4], trajectory.a[5])
        # for
        #     max_t = distance / self.max_velocity
        #
        #
        #     #
        #     # end = []
        #     #
        #     # total_distance = min(self.distance(waypoints, 0, len(waypoints) - 1), MAX_T*self.max_velocity)
        # else:
        #     # speed at traffic light and after should be 0
        #     # total_distance = min(self.distance(waypoints, 0, traffic_light), MAX_T*self.max_velocity)
        #     distance = self.distance(waypoints, 0, traffic_light)
        # max_t = distance / self.max_velocity
        #
        # pass


    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist