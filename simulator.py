import math
import numpy as np
import matplotlib.pyplot as plt

from kayak import Kayak
from weather import Weather
from autopilot import Autopilot
from angular_tools import shortest_angle_difference




def simulate_heading_control(kayak, autopilot, weather, target_heading, plot=False, target_power=0, simulated_time=60, timestep=0.03):
    # Simulation parameters
    time = 0  # starting time in s

    # Compile parameters
    times = []
    angles = []
    errors = []
    angular_velocities = []
    left_engine_powers = []
    right_engine_powers = []

    # Simulate
    for step in np.arange(0, np.round(simulated_time / timestep)):
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
        #ax1.plot(times, angular_velocities, 'k--', label='Angular velocities')
        ax1.axhline(y=target_heading, color='g', linestyle='--', linewidth=3, label='Target')
        ax1.axvline(x=10, color='k', linestyle='-', linewidth=5, alpha=0.1)
        ax1.set_ylabel('Heading (deg)')
        ax1.set_title(f'kp={autopilot.kp}, kd={autopilot.kd}')
        ax1.set_ylim([np.min(angles)-10, np.max(angles)+10])

        # If there are any wind stable points within the plot, draw them
        if weather.wind_speed != 0:
            stable_wind_points = [(weather.wind_heading + 90) % 360, (weather.wind_heading - 90) % 360]
            for stable_wind_point in stable_wind_points:
                if np.min(angles)-10 < stable_wind_point < np.max(angles)+10:
                    ax1.axhline(y=stable_wind_point, color='r', linewidth=10, alpha=0.1, linestyle='-', label='Wind attractor')

        ax1.legend()

        # Add your new subplot on the second plot (you can modify this as needed)
        # For example, you can plot angular_velocity on this subplot.
        ax2.plot(times, right_engine_powers, color='r', label='Right')
        ax2.plot(times, left_engine_powers,  color='b', label='Left')
        ax2.axvline(x=10, color='k', linestyle='-', linewidth=5, alpha=0.1)
        ax2.set_ylabel('Engine Power (%)')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylim([-5,105])
        ax2.set_xlim([0,np.max(times)])
        ax2.legend()
        plt.tight_layout()
        plt.show()

    return times, angles, angular_velocities, left_engine_powers, right_engine_powers



target_heading = 35    # deg
target_power   = 50    # 0-200%
kayak_heading  = 125   # deg
simulated_time = 400   # s
timestep       = 0.03  # s

weather = Weather(wind_speed=10, wind_heading=170) # make it want to go to 80
kayak = Kayak(weather=weather, initial_speed=0, initial_angle=kayak_heading)

autopilot = Autopilot(kp=16, kd=45, ki=0, target_power=target_power,
                      target_heading=target_heading) # kp 16 and kd 45 is a good start, set to 0 for no autopilot
autopilot = Autopilot(kp=0, kd=0, ki=0, target_power=target_power,
                      target_heading=target_heading) # kp 16 and kd 45 is a good start, set to 0 for no autopilot

simulate_heading_control(kayak=kayak, autopilot=autopilot, weather=weather,
                         target_heading=target_heading, plot=True, target_power=target_power,
                         simulated_time=simulated_time)

# Fastest time for 90 deg turn is 8.80 s

# KP 10, KD 85
