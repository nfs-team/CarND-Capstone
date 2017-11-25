from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband,
		 decel_limit, accel_limit, wheel_radius, wheel_base,
		 steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: car params:
	self.vehicle_mass = vehicle_mass
	self.fuel_capacity = fuel_capacity
	self.brake_deadband = brake_deadband
	self.decel_limit = decel_limit
	self.accel_limit = accel_limit
	self.wheel_base = wheel_base
	self.wheel_radius = wheel_radius
	self.steer_ratio = steer_ratio
	self.max_lat_accel = max_lat_accel
	self.max_steer_angle = max_steer_angle

	self.max_speed = 20
	self.min_speed = 0

        self.yaw_controller = YawController(wheel_base, steer_ratio, self.min_speed, max_lat_accel, max_steer_angle)
	self.pid = PID(kp=2.5, ki=0.0, kd=1.2, mn=self.decel_limit, mx=self.accel_limit)
	self.lowpass = LowPassFilter(0.2, 1.0)

    def control(self, proposed_linear_velocity, proposed_angular_velocity,
		current_linear_velocity, current_angular_velocity, time_step=None):
        # TODO: Why do we even need angular velocity?

        proposed_angular_velocity = self.lowpass.filt(proposed_angular_velocity)
	steer = self.yaw_controller.get_steering(proposed_linear_velocity, proposed_angular_velocity,
                                                 current_linear_velocity)

       	v_delta = proposed_linear_velocity - current_linear_velocity
	throttle = self.pid.step(v_delta, time_step)
	brake = 0.0

	# we want to brake
	if throttle < 0.0:
	    brake = abs((self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * throttle )
	    throttle = 0.0

        return throttle, brake, steer
