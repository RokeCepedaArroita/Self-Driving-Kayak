# CONTROL CODE BELOW


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
        # Calculate the integral term
        self.integral += error * dt
        # Calculate the derivative term
        derivative = (error - self.prev_error) / dt
        # Calculate the control output
        control_output = self.kp * error + self.ki * self.integral + self.kd * derivative
        # Update the previous error for the next iteration
        self.prev_error = error
        return control_output



def control(kp, ki, kd, desired_heading, heading, angular_velocity):

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
    total_time = 300  # s
    timestep = 0.01  # s
    time = 0  # starting time in s

    # Initialize the PID controller for both engines
    left_engine_pid = PIDController(kp, ki, kd)
    right_engine_pid = PIDController(kp, ki, kd)

    # Compile parameters
    times = []
    angles = []
    angular_velocities = []
    left_engine_powers = []
    right_engine_powers = []

    # Simulate
    for step in np.arange(0, np.round(total_time / timestep)):
        # Control
        left_engine_power, right_engine_power = control(
            kp, ki, kd, desired_heading, kayak.angle, kayak.angular_velocity
        )
        # Effect
        angle, angular_velocity = kayak.update(left_engine_power, right_engine_power, dt=timestep)
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

    return times, angles, angular_velocities, left_engine_powers, right_engine_powers


desired_heading = 180
# kayak = Kayak()
# simulate_heading_control(kp=10, ki=0.0, kd=10, kayak=kayak, desired_heading=desired_heading, plot=True)
