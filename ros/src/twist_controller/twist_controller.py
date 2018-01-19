import math
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_SPEED = 0


class Controller(object):
    """Controller"""
    def __init__(self, **parameters):

        self.parameters = parameters

        self.speed_pid = PID(2.0, 0.02, 0.02,
                             parameters['decel_limit'],
                             parameters['accel_limit'])

        # self.steer_pid = PID(0.5, 0.0, 0.02,
        #                      -parameters['max_steer_angle'],
        #                      parameters['max_steer_angle'])

        self.yaw_control = YawController(parameters['wheel_base'],
                                         parameters['steer_ratio'],
                                         MIN_SPEED,
                                         parameters['max_lat_accel'],
                                         parameters['max_steer_angle'])

        # TODO Tune the PID and see if we need to use filter
        self.t_lpf = LowPassFilter(tau = 3, ts = 1)
        self.s_lpf = LowPassFilter(tau = 3, ts = 1)

    def control(self, twist_cmd, current_velocity, time_interval):
        """control
        :param twist_cmd:
        :param current_velocity:
        :param time_interval:
        """
        # Calculate throttle
        target_velocity = twist_cmd.twist.linear.x
        real_velocity = current_velocity.twist.linear.x
        vel_err = target_velocity - real_velocity

        throttle = self.speed_pid.step(vel_err, time_interval)
        throttle = self.t_lpf.filt(throttle)

        # TODO Need to modify to use CTE
        # Calculate steering
        angular_velocity = twist_cmd.twist.angular.z
        target_steering = self.yaw_control.get_steering(target_velocity,
                                                        angular_velocity,
                                                        real_velocity)
        target_steering = self.s_lpf.filt(target_steering)

        # Calculate brake
        if throttle > 0.0:
            brake = 0.0
        else:
            decel = abs(throttle)
            if decel < self.parameters['brake_deadband']:
                decel = 0.0
            brake = decel * (self.parameters['vehicle_mass'] + self.parameters['fuel_capacity'] * GAS_DENSITY) * self.parameters['wheel_radius']

            throttle = 0.0

        return throttle, brake, target_steering

    def reset(self):
        self.speed_pid.reset()
