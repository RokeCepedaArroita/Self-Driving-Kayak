from angular_tools import shortest_angle_difference

class Autopilot:
    def __init__(self, kp, ki, kd, target_power=0, target_heading=None):

        # PID control loop parameters
        self.kp = kp  # proportional gain
        self.ki = ki  # integral gain
        self.kd = kd  # derivative gain
        self.integral = 0 # initialize integral term

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
            #     # Calculate the integral term
            #     self.integral += error * dt

            # If error positive, left engine to full, otherwise reverse
            left_engine_power  = +self.kp*error  - self.kd*angular_velocity # + self.ki*integral_term
            right_engine_power = -self.kp*error  + self.kd*angular_velocity # + self.ki*integral_term

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




        return left_engine_power, right_engine_power
