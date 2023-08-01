import math
import numpy as np
import matplotlib.pyplot as plt

class Kayak:
    def __init__(self, length=3.65, width=1.08, mass=150, engine_spacing=0.25, max_thrust=2.5, initial_angle=90, initial_angular_velocity=0):
        self.length = length
        self.width = width
        self.mass = mass
        self.engine_spacing = engine_spacing
        self.max_thrust = max_thrust
        self.angle = initial_angle
        self.angular_velocity = initial_angular_velocity

        # Calculate moment of inertia
        self.I = (1/12) * self.mass * (self.length**2 + self.width**2)


    def update(self, left_engine_power, right_engine_power, dt):
        # Convert engine power from percentage to actual thrust
        left_thrust = left_engine_power / 100 * self.max_thrust
        right_thrust = right_engine_power / 100 * self.max_thrust

        # Calculate the net torque, positive is clockwise
        net_torque = (left_thrust - right_thrust) * self.engine_spacing

        # Calculate angular acceleration in deg/s**2, positive is to the rights
        angular_acceleration = np.rad2deg(net_torque / self.I)

        # Update the angular velocity in deg/s
        self.angular_velocity = self.angular_velocity + angular_acceleration*dt

        # Get average angular velocity during this step in deg/s -- it's close enough
        average_angular_velocity = self.angular_velocity - angular_acceleration*dt/2

        # Update the angle in deg, use the average
        self.angle = self.angle + average_angular_velocity*dt + 0.5*angular_acceleration*dt**2

        # Return the new angle and angular velocity
        return self.angle, self.angular_velocity





def simple_control(power_setting, desired_heading, heading, angular_velocity):

    # Measure the angle difference, positive means we have to turn right
    angle_difference = desired_heading - heading

    # If positive, left engine to full, otherwise reverse, regardless of velocity
    if angle_difference > 0:
        left_engine_power = 100
        right_engine_power = 0
    elif angle_difference < 0:
        right_engine_power = 100
        left_engine_power = 0
    else:
        right_engine_power = 50
        left_engine_power = 50



    return left_engine_power, right_engine_power






class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.prev_error = 0
        self.integral = 0

    def update(self, setpoint, current_value, dt):
        # Calculate the error
        error = setpoint - current_value

        # Calculate the integral term using the trapezoidal rule
        self.integral += (error + self.prev_error) * dt / 2.0

        # Calculate the derivative term
        derivative = (error - self.prev_error) / dt

        # Calculate the control output
        control_output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Update the previous error for the next iteration
        self.prev_error = error

        return control_output

def shortest_angle_difference(current_heading, desired_heading):
    # Ensure both angles are within the range [0, 360)
    current_heading %= 360
    desired_heading %= 360

    # Calculate the raw difference between the angles
    raw_difference = desired_heading - current_heading

    # Adjust the raw difference to the shortest angle (considering the wrap-around)
    if abs(raw_difference) <= 180:
        # The difference is already the shortest angle
        return raw_difference
    elif raw_difference > 180:
        # The shortest angle is by turning left (negative)
        return raw_difference - 360
    else:
        # The shortest angle is by turning right (positive)
        return raw_difference + 360



def sophisticated_control(kp, ki, kd, desired_heading, heading, angular_velocity):

    # Time step for PID calculation (you may need to adjust this based on your application)
    dt = 0.1

    # Initialize the PID controller for both engines
    left_engine_pid = PIDController(kp, ki, kd)
    right_engine_pid = PIDController(kp, ki, kd)

    # Measure the angle difference, positive means we have to turn right
    angle_difference = shortest_angle_difference(heading, desired_heading)

    # Calculate the control output for both engines using PID controller
    left_engine_power = left_engine_pid.update(angle_difference, 0, dt)
    right_engine_power = right_engine_pid.update(-angle_difference, 0, dt)

    # Scale the control outputs to match your engine power range
    max_power = 100
    left_engine_power = max(min(left_engine_power, max_power), 0)
    right_engine_power = max(min(right_engine_power, max_power), 0)

    return left_engine_power, right_engine_power







def simulate_heading_control(kp, ki, kd, kayak, desired_heading, plot=False):

    # Simulation parameters
    total_time = 300 # s
    timestep = 0.01 # s
    time = 0 # starting time in s

    # Compile parameters
    times = []
    angles = []
    angular_velocities = []
    left_engine_powers = []
    right_engine_powers = []

    # Simulate
    for step in np.arange(0,np.round(total_time/timestep)):
        # Control
        left_engine_power, right_engine_power = sophisticated_control(kp, ki, kd, desired_heading, kayak.angle, kayak.angular_velocity)
        # Effect
        angle, angular_velocity = kayak.update(left_engine_power, right_engine_power, dt=timestep)
        if step == 0:
            initial_angle = angle
        time = time + timestep
        angles.append(angle)
        angular_velocities.append(angular_velocity)
        times.append(time)
        left_engine_powers.append(left_engine_power)
        right_engine_powers.append(right_engine_power)


    if plot:
        # Plot
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))

        # Plot the angles on the first subplot
        ax1.plot(times, angles)
        ax1.axhline(y=desired_heading, color='g', linestyle='--')
        ax1.set_ylabel('Angle (deg)')
        ax1.set_title(f'Kp={kp}, Kd={kd}, Ki={ki}')

        # Add your new subplot on the second plot (you can modify this as needed)
        # For example, you can plot angular_velocity on this subplot.
        ax2.plot(times, right_engine_powers, color='r', label="Right")
        ax2.plot(times, left_engine_powers, color='b', label="Left")
        ax2.set_ylabel('Engine Power (%)')
        ax2.set_xlabel('Time')
        ax2.legend()
        plt.tight_layout()
        plt.show()

    return times, angles


desired_heading = 180
# kayak = Kayak()
# simulate_heading_control(kp=10, ki=0.0, kd=10, kayak=kayak, desired_heading=desired_heading, plot=True)
