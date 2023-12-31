from angular_tools import shortest_angle_difference
import numpy as np

class Autopilot:
    def __init__(self, kp, ki, kd, timestep=0.122, smoothing_time=0, derivative_smoothing_time=0,
                 alpha_fading_memory = 1.0, target_power=0, target_heading=None, deadband=None,
                 sensor_noise=None, integral_activation_angle=20):

        from kalmanfilter import KalmanFilter

        # PID control loop parameters
        self.kp = kp  # proportional gain
        self.ki = ki  # integral gain
        self.kd = kd  # derivative gain

        # PID smoothing parameters
        self.timestep                  = timestep                  # s, time equivalent to the control frequency
        self.smoothing_time            = smoothing_time            # s, typical output smoothing time
        self.derivative_smoothing_time = derivative_smoothing_time # s, typical derivative smoothing time
        self.alpha_fading_memory       = alpha_fading_memory       # memory fading parameter - higher values result in less lag and higher noise
        self.integral_activation_angle = integral_activation_angle # when we are within this error in deg from the setpoint, the integral starts counting

        # Sensor noise estimates
        if sensor_noise is None:
            self.sensor_noise = {'compass_heading': 5,   # deg, 1 standard deviation
                                 'angular_speed'  : 9}  # deg/s, 1 standard deviation
        else:
            self.sensor_noise = sensor_noise

        # Initialize variables
        self.integral_is_counting = False      # whether integral activated
        self.integral = 0                      # integral term
        self.derivative = 0                    # previous derivative term
        self.previous_left_engine_power  = 0   # previous left power
        self.previous_right_engine_power = 0   # previous right power

        # Kayak autopilot settings
        self.target_power    = target_power     # combined power level % (0-200)
        self.target_heading  = target_heading   # desired heading in deg (0-360)
        self.deadband        = deadband         # minimum change in the motors for a change to occur

        # Create covariance matrix
        self.covariance_matrix = [  [ sensor_noise['compass_heading']**2, sensor_noise['covariance']      ],
                                    [ sensor_noise['covariance']        , sensor_noise['angular_speed']**2]  ]

        # Initialize Kalman Filter
        self.kalmanfilter = KalmanFilter(self.covariance_matrix, alpha_fading_memory)



    def PIDController(self, current_heading, angular_velocity):
        ''' Use a PID control loop to choose appropiate left and right engine powers '''

        # Maximum power on each engine (minimum is 0)
        max_power = 100


        if self.target_heading is not None: # If autopilot is on

            # Filter sensor readings with Kalman Filter
            self.kalmanfilter.predict(self.timestep)
            self.kalmanfilter.update([current_heading, angular_velocity])

            # Get filtered readings
            self.filtered_heading, self.filtered_angular_velocity = self.kalmanfilter.x

            # Measure the angle difference, positive means we have to turn right: use filtered readings in PID control loop
            error = shortest_angle_difference(self.filtered_heading, self.target_heading)

            # Check if we are within the integral activation range
            if self.integral_is_counting == False:
                if np.abs(error) < self.integral_activation_angle:
                    self.integral_is_counting = True

            # Integral calculations
            if self.ki != 0 and self.integral_is_counting: # if the integral term is active
                # Saturation handling: if any of the motors is at 100%, do not increase the integral term
                if self.previous_left_engine_power!=max_power and self.previous_left_engine_power!=max_power and self.ki!=0:
                    self.integral += error * self.timestep
                    # Clamp the values of the integral to represent only a maximum autority of 100%
                    self.integral = max(min(self.integral, max_power/self.ki), -max_power/self.ki)
                else: # if the cause of the saturation is the integral, lower it
                    if self.integral >= max_power/self.ki:
                        self.integral = 0  # reset it


            # Smooth derivative term to reduce noise in the output
            if self.derivative_smoothing_time > 0:
                alpha = 1 - np.exp(-self.timestep/self.derivative_smoothing_time)
                self.derivative  = alpha * self.filtered_angular_velocity  + (1 - alpha) * self.derivative
            else:
                self.derivative = self.filtered_angular_velocity


            # If error positive, left engine to full, otherwise reverse
            left_engine_power  = +self.kp*error  - self.kd*self.derivative + self.ki*self.integral
            right_engine_power = -self.kp*error  + self.kd*self.derivative - self.ki*self.integral

            # TODO: Deadband: if change is small, don't change outputs
            if self.deadband is not None:
                if max(np.abs(left_engine_power-self.previous_left_engine_power),
                    np.abs(right_engine_power-self.previous_right_engine_power)) < self.deadband:
                    # Keep old values
                    left_engine_power = self.previous_right_engine_power
                    right_engine_power = self.previous_left_engine_power

            # Smooth output with a low pass filter
            if self.smoothing_time > 0:
                # This filter smooths out output varyations in timescales less than self.smoothing_time seconds.
                # This results on less wear and tear of the motors due to large and frequent variations, at the
                # expense of a slower response time. If alpha = 1: no smoothing. If alpha=0: infinite smoothing
                # so no output variations are allowed.
                alpha = 1 - np.exp(-self.timestep/self.smoothing_time)
                left_engine_power  = alpha * left_engine_power  + (1 - alpha) * self.previous_left_engine_power
                right_engine_power = alpha * right_engine_power + (1 - alpha) * self.previous_right_engine_power


        else: # If target heading is None, the autopilot is off and does not make corrections
            left_engine_power, right_engine_power = 0, 0


        # Normalize output from 0% to 100% power on each engine to add forward thrust
        left_engine_power  = max(min(left_engine_power,  max_power), 0)
        right_engine_power = max(min(right_engine_power, max_power), 0)



        # Adjust output to maintain some minimum value of forward thrust at all times
        if left_engine_power+right_engine_power < self.target_power:
            # Calculate the combined power deficit (e.g. 50 means that each engine should be running 25% higher)
            power_deficit = self.target_power - (left_engine_power+right_engine_power)
            # Power available in the engine that is using the most power
            available_power = max_power - max(left_engine_power, right_engine_power)
            # Maintain PID difference while adding the fraction of available_power needed to both engines
            left_engine_power  = left_engine_power  + min(available_power, power_deficit/2)
            right_engine_power = right_engine_power + min(available_power, power_deficit/2)


        # Copy previous engine powers
        self.previous_left_engine_power = left_engine_power
        self.previous_right_engine_power = right_engine_power


        return left_engine_power, right_engine_power
