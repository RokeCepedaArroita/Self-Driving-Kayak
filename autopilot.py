from angular_tools import shortest_angle_difference
import numpy as np

class Autopilot:
    def __init__(self, kp, ki, kd, timestep=0.08, smoothing_time=0, target_power=0, target_heading=None):

        # PID control loop parameters
        self.kp = kp  # proportional gain
        self.ki = ki  # integral gain
        self.kd = kd  # derivative gain

        # PID smoothing parameters
        self.smoothing_time = smoothing_time # s, typical output smoothing time
        self.timestep       = timestep       # s, time equivalent to the control frequency

        # Initialize variables
        self.integral = 0 # integral term
        self.previous_left_engine_power  = 0 # previous left power
        self.previous_right_engine_power = 0 # previous right power

        # Kayak autopilot settings
        self.target_power    = target_power     # combined power level % (0-200)
        self.target_heading  = target_heading   # desired heading in deg (0-360)



    def PIDController(self, current_heading, angular_velocity):
        ''' Use a PID control loop to choose appropiate left and right engine powers '''

        # Maximum power on each engine (minimum is 0)
        max_power = 100

        if self.target_heading is not None: # If autopilot is on

            # Measure the angle difference, positive means we have to turn right
            error = shortest_angle_difference(current_heading, self.target_heading)

            # TODO: Integral term, and reset it from time to time or have a moving integral maybe??
            # Calculate the integral term.

            # ONLY CONSIDER THE LAST 10 SECONDS OF THE INTEGRAL!!


            if self.previous_left_engine_power!=max_power and self.previous_left_engine_power!=max_power:
                # Only add to the integral term if we are at a steady-ish state?
                self.integral += error * self.timestep
                # Clamp the values of the integral to represent only a maximum autority of 100%
                self.integral = max(min(self.integral,  max_power/self.ki), -max_power/self.ki)

                

            # Saturation handling: if any of the motors is at 100%, do not increase the integral term


            # Add clamping so that the integral term does not increase if the motors are saturated



            # TODO: To add a deadband, you can modify the P component as follows:
            #
            # if |error| <= deadband:
            # P = 0
            # else:
            # P = Kp * error


            # If error positive, left engine to full, otherwise reverse
            left_engine_power  = +self.kp*error  - self.kd*angular_velocity + self.ki*self.integral
            right_engine_power = -self.kp*error  + self.kd*angular_velocity - self.ki*self.integral

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
