import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
        accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel,
        max_steer_angle):
        # Yaw Controller for steering prediction
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1,
            max_lat_accel, max_steer_angle)
        # PID parameters
        kp = 0.35
        ki = 0.00001
        kd = 0.
        mn = 0.  # Minimum throttle value
        mx = 0.2 # Maximum throttle value
        # PID Controller for throttle prediction
        self.throttle_controller = PID(kp, ki, kd, mn, mx)

        # Low Pass Filter
        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = 0.02 # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        # Max braking torque to be applied while decelerating
        vehicle_mass_with_fuel = vehicle_mass + fuel_capacity * GAS_DENSITY
        self.brake_torque = vehicle_mass_with_fuel * wheel_radius
        self.decel_limit = decel_limit

        self.last_time = rospy.get_time()


    def control(self, dbw_enabled, angular_vel, linear_vel, current_vel):
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        # Calculate velocity error
        current_vel = self.vel_lpf.filt(current_vel)
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        # Calculate sample time or delta_t
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        # Get throttle value from PID Controller
        throttle = self.throttle_controller.step(vel_error, sample_time)

        if vel_error < 0.:
            # Decelerate if current vel > target vel
            throttle = 0.0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.brake_torque
        else:
            # Throttle to accelerate should be between 0.0 and 1.0.
            throttle = min(1.0, throttle)
            brake = 0.0

        # Get steering value from Yaw Controller
        steer = self.yaw_controller.get_steering(linear_vel, angular_vel,
            current_vel)

        return throttle, brake, steer
