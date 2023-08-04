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
    speeds = []
    accelerations = []
    errors = []
    angular_velocities = []
    left_engine_powers = []
    right_engine_powers = []

    # Simulate
    for step in np.arange(0, np.round(simulated_time / timestep)):
        # Control
        left_engine_power, right_engine_power = autopilot.PIDController(kayak.angle, kayak.angular_velocity)
        # Effect
        angle, angular_velocity, position, speed, acceleration = kayak.update(left_engine_power, right_engine_power, dt=timestep)
        # Advance time
        time = time + timestep
        # Append all the values
        times.append(time)
        angles.append(angle)
        angular_velocities.append(angular_velocity)
        left_engine_powers.append(left_engine_power)
        right_engine_powers.append(right_engine_power)
        speeds.append(speed)
        accelerations.append(acceleration)

    if plot:
        # Plot
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(9, 9), gridspec_kw={'height_ratios': [1, 2, 1]})

        # Plot speed and acceleration on the first subplot
        maximum_speed = weather.maximum_speed(target_power, kayak.power_to_thrust, kayak.CdA)
        ax1.plot(times, speeds, color='C0', linewidth=3, label='Speed (km/h)')
        ax1.plot(times, np.multiply(accelerations,3.6*6), color='C3', linewidth=3, label='Acceleration (6*(km/h)/s)')
        ax1.axhline(y=maximum_speed, color='C0', linestyle='-', linewidth=10, alpha=0.15, label='Max Target Speed')
        ax1.set_ylabel('Speed and Acceleration')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylim([0,7])
        ax1.set_xlim([0,np.max(times)])
        ax1.set_title(f'kp={autopilot.kp}, kd={autopilot.kd}')
        ax1.legend()

        # Plot the angles on the second subplot
        ax2.plot(times, angles, label='Kayak', linewidth=5)
        #ax2.plot(times, angular_velocities, 'k--', label='Angular velocities')
        ax2.axhline(y=target_heading, color='g', linestyle='--', linewidth=3, label='Target')
        ax2.axvline(x=10, color='k', linestyle='-', linewidth=5, alpha=0.1)
        ax2.set_ylabel('Heading (deg)')
        ax2.set_ylim([np.min(angles)-10, np.max(angles)+10])

        # If there are any wind stable points within the plot, draw them
        if weather.wind_speed != 0:
            stable_wind_points = [(weather.wind_heading + 90) % 360, (weather.wind_heading - 90) % 360]
            for stable_wind_point in stable_wind_points:
                if np.min(angles)-10 < stable_wind_point < np.max(angles)+10:
                    ax2.axhline(y=stable_wind_point, color='r', linewidth=10, alpha=0.1, linestyle='-', label='Wind attractor')

        ax2.legend()


        # Plot engine powers on the third subplot
        ax3.plot(times, right_engine_powers, color='r', label='Right')
        ax3.plot(times, left_engine_powers,  color='b', label='Left')
        ax3.axvline(x=10, color='k', linestyle='-', linewidth=5, alpha=0.1)
        ax3.set_ylabel('Engine Power (%)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylim([-5,105])
        ax3.set_xlim([0,np.max(times)])
        ax3.legend()
        plt.tight_layout()
        plt.show()

    return times, angles, angular_velocities, left_engine_powers, right_engine_powers



target_power   = 100    # 0-200%
wind_speed     = 10

target_heading = 35    # deg
kayak_heading  = 125   # deg



simulated_time = 200   # s
timestep       = 0.03  # s

weather = Weather(wind_speed=wind_speed, wind_heading=170) # make it want to go to 80
kayak = Kayak(weather=weather, initial_speed=0, initial_angle=kayak_heading)

autopilot = Autopilot(kp=16, kd=45, ki=0, target_power=target_power,
                      target_heading=target_heading) # kp 16 and kd 45 is a good start, set to 0 for no autopilot
# autopilot = Autopilot(kp=0, kd=0, ki=0, target_power=target_power,
#                       target_heading=target_heading) # kp 16 and kd 45 is a good start, set to 0 for no autopilot

simulate_heading_control(kayak=kayak, autopilot=autopilot, weather=weather,
                         target_heading=target_heading, plot=True, target_power=target_power,
                         simulated_time=simulated_time)

# Fastest time for 90 deg turn is 8.80 s

# KP 10, KD 85
