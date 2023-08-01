from angular_tools import shortest_angle_difference

class Autopilot:
    def __init__(self, kp, ki, kd, target_power=0, target_heading=None):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.target_power    = target_power     # Combined power level % (0-200)
        self.target_heading  = target_heading   # Desired heading in deg (0-360)
        self.prev_error = 0
        self.integral = 0

    # def update(self, setpoint, current_value, dt):
    #     # Calculate the error
    #     error = setpoint - current_value
    #     # Calculate the integral term
    #     self.integral += error * dt
    #     # Calculate the derivative term
    #     derivative = (error - self.prev_error) / dt
    #     # Calculate the control output
    #     control_output = self.kp * error + self.ki * self.integral + self.kd * derivative
    #     # Update the previous error for the next iteration
    #     self.prev_error = error
    #     return control_output


    def PIDController(self, current_heading, angular_velocity):
        ''' Use a PID control loop to choose appropiate left and right engine powers '''

        # TODO: If target heading is None, just sent the same power to both engines

        # Measure the angle difference, positive means we have to turn right
        error = shortest_angle_difference(current_heading, self.target_heading)

        # If error positive, left engine to full, otherwise reverse
        left_engine_power  = self.kp*error  - self.kd*(angular_velocity)
        right_engine_power = self.kp*-error + self.kd*(angular_velocity)

        # TODO: Integral term, and reset it from time to time or have a moving integral maybe??

        # TODO: Optimization under different conditions, and can model wind too!

        # Normalize output from 0% to 100% power on each engine
        max_power = 100
        left_engine_power  = max(min(left_engine_power,  max_power), 0)
        right_engine_power = max(min(right_engine_power, max_power), 0)

        # Adjust output to maintain minimum power value TODO: FIX SPIKES!!!
        if left_engine_power+right_engine_power < self.target_power:
            # Calculate the combined power deficit (e.g. 50 means that each engine should be running 25% higher)
            power_deficit = self.target_power - (left_engine_power+right_engine_power)

            # Power available in the engine that is using the most power
            available_power = 100 - max(left_engine_power, right_engine_power)

            # Maintain PID difference while adding the fraction of available_power needed to both engines
            left_engine_power  = left_engine_power  + min(available_power,power_deficit/2)
            right_engine_power = right_engine_power + min(available_power,power_deficit/2)



        return left_engine_power, right_engine_power
