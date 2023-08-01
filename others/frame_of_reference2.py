import numpy as np
import matplotlib.pyplot as plt

def calculate_apparent_wind(kayak_speed, kayak_heading, wind_speed, wind_heading):
    # Convert headings to radians
    kayak_heading_rad = np.deg2rad(360 - kayak_heading)
    wind_heading_rad = np.deg2rad(360 - wind_heading)

    # Convert speeds and headings to Cartesian coordinates
    kayak_vx = kayak_speed * np.cos(kayak_heading_rad)
    kayak_vy = kayak_speed * np.sin(kayak_heading_rad)
    wind_vx = wind_speed * np.cos(wind_heading_rad)
    wind_vy = wind_speed * np.sin(wind_heading_rad)

    # Calculate apparent wind speed and direction
    apparent_vx = wind_vx - kayak_vx
    apparent_vy = wind_vy - kayak_vy
    apparent_speed = np.sqrt(apparent_vx**2 + apparent_vy**2)
    apparent_angle = (360 - np.rad2deg(np.arctan2(apparent_vy, apparent_vx))) % 360

    # Return apparent wind speed and direction relative to the kayak
    return apparent_speed, apparent_angle - kayak_heading


import math
import matplotlib.pyplot as plt

def plot_compass(kayak_speed, kayak_heading, wind_speed, wind_heading, apparent_speed, apparent_angle):
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

    # Adjust the compass so the kayak is always heading North
    #ax.set_theta_zero_location('N')
    #ax.set_theta_direction(-1)

    # Normalize speeds for plotting
    kayak_speed_normalized = kayak_speed / max(kayak_speed, wind_speed, apparent_speed)
    wind_speed_normalized = wind_speed / max(kayak_speed, wind_speed, apparent_speed)
    apparent_speed_normalized = apparent_speed / max(kayak_speed, wind_speed, apparent_speed)

    # Plot the kayak direction, wind direction, and apparent wind direction
    # Plot kayak's velocity
    kayak_heading_rad = math.radians(kayak_heading)
    kayak_velocity_x = kayak_speed_normalized * math.cos(kayak_heading_rad)
    kayak_velocity_y = kayak_speed_normalized * math.sin(kayak_heading_rad)
    kayak_arrow = ax.arrow(0, 0, kayak_velocity_x, kayak_velocity_y, head_width=0.1, head_length=0.1, fc='b', ec='b',
                           label="Kayak's Velocity\nMagnitude: {:.2f} knots".format(kayak_speed))

    # Plot wind's velocity
    wind_direction_rad = math.radians(wind_heading)
    wind_velocity_x = wind_speed_normalized * math.cos(wind_direction_rad)
    wind_velocity_y = wind_speed_normalized * math.sin(wind_direction_rad)
    wind_arrow = ax.arrow(0, 0, wind_velocity_x, wind_velocity_y, head_width=0.1, head_length=0.1, fc='r', ec='r',
                          label="Wind's Velocity\nMagnitude: {:.2f} knots".format(wind_speed))

    # Plot relative wind's velocity
    apparent_direction_rad = math.radians(apparent_angle)
    apparent_velocity_x = apparent_speed_normalized * math.cos(apparent_direction_rad)
    apparent_velocity_y = apparent_speed_normalized * math.sin(apparent_direction_rad)
    apparent_arrow = ax.arrow(0, 0, apparent_velocity_x, apparent_velocity_y, head_width=0.1, head_length=0.1, fc='g', ec='g',
                              label="Apparent Wind's Velocity\nMagnitude: {:.2f} knots".format(apparent_speed))


    # Add a legend and show the plot
    ax.legend()
    plt.show()

# Test the functions
kayak_speed = 5
kayak_heading = 0
wind_speed = 6
wind_heading = 90

apparent_speed, apparent_angle = calculate_apparent_wind(kayak_speed, kayak_heading, wind_speed, wind_heading)
plot_compass(kayak_speed, kayak_heading, wind_speed, wind_heading, apparent_speed, apparent_angle)
