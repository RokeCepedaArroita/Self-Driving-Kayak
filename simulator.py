import math
import numpy as np
import matplotlib.pyplot as plt

from kayak import Kayak
from weather import Weather
from autopilot import Autopilot
from angular_tools import shortest_angle_difference




def simulate_heading_control(kayak, autopilot, target_heading, plot=False, target_power=0):
    # Simulation parameters
    total_time = 400  # s
    timestep = 0.01  # s
    time = 0  # starting time in s

    # Compile parameters
    times = []
    angles = []
    errors = []
    angular_velocities = []
    left_engine_powers = []
    right_engine_powers = []

    # Simulate
    for step in np.arange(0, np.round(total_time / timestep)):
        # Control
        left_engine_power, right_engine_power = autopilot.PIDController(kayak.angle, kayak.angular_velocity)
        # Effect
        angle, angular_velocity = kayak.update(left_engine_power, right_engine_power, dt=timestep)
        # Append all the values
        time = time + timestep
        error = shortest_angle_difference(angle, target_heading)
        angles.append(angle)
        angular_velocities.append(angular_velocity)
        times.append(time)
        left_engine_powers.append(left_engine_power)
        right_engine_powers.append(right_engine_power)
        errors.append(error)

    if plot:
        # Plot
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))

        # Plot the angles on the first subplot
        ax1.plot(times, angles, label='Kayak', linewidth=5)
        #ax1.plot(times, errors, 'k--', label='Errors')
        ax1.axhline(y=target_heading, color='g', linestyle='--')
        ax1.set_ylabel('Angle (deg)')
        ax1.legend()
        ax1.set_title(f'kp={autopilot.kp}, kd={autopilot.kd}')

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



target_heading = 100
target_power = 80

weather = Weather()
kayak = Kayak()
autopilot = Autopilot(kp=6, kd=70, ki=0, target_power=50, target_heading=target_heading) # kp 6 and kd 70 good start
weather = None

simulate_heading_control(kayak=kayak, autopilot=autopilot, target_heading=target_heading, plot=True, target_power=target_power)

# Fastest time for 90º turn is 30.215 s

# KP 10, KD 85
