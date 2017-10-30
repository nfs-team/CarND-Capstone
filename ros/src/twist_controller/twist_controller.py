from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        pass

    def control(self, proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        steer = self.yaw_controller.get_steering(proposed_linear_velocity, proposed_angular_velocity,
                                                 current_linear_velocity)
        return .1, 0., steer
