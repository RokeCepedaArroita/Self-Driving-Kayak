import math
import matplotlib.pyplot as plt
import numpy as np

def apparent_wind(kayak_heading, kayak_speed, wind_heading, wind_speed):
    ''' Calculate the apparent wind felt in the kayak as a result of the combined motion of
        the kayak and the wind. Output the apparent wind amplitude and give the direction in
        relative terms to the nose of the kayak:

        0   deg  -> head-on wind
        90  deg  -> right to left
        180 deg  -> full tailwind
        270 deg  -> left to right

        '''

    # The apparent wind caused by the kayak's movement is the same as turning the kayak vector 180 deg
    # (e.g. as if the kayak was stationary and the wind hit it in the opposite direction). We can achieve
    # this in two ways, either by rotating 180 deg or by setting the magnitude to the opposite sign
    kayak_wind_speed = -kayak_speed # wind caused by kayak movement acts in the opposite direction

    # Convert headings to radians
    kayak_heading_rad = np.deg2rad(kayak_heading)
    wind_heading_rad = np.deg2rad(wind_heading)

    # Convert speeds and headings to Cartesian coordinates
    kayak_vx = kayak_wind_speed * np.cos(kayak_heading_rad)
    kayak_vy = kayak_wind_speed * np.sin(kayak_heading_rad)
    wind_vx = wind_speed * np.cos(wind_heading_rad)
    wind_vy = wind_speed * np.sin(wind_heading_rad)

    # Calculate apparent wind speed and direction
    apparent_vx = wind_vx + kayak_vx
    apparent_vy = wind_vy + kayak_vy
    apparent_speed = np.sqrt(apparent_vx**2 + apparent_vy**2)
    apparent_angle = np.rad2deg(np.arctan2(apparent_vy, apparent_vx))

    # Calculate the apparent wind angle relative to the kayak
    apparent_angle_relative_to_kayak = apparent_angle - kayak_heading
    apparent_angle_relative_to_kayak %= 360

    # Return apparent wind speed and direction relative to the kayak
    return apparent_speed, apparent_angle





def plot_apparent_wind(kayak_heading, kayak_speed, wind_direction, wind_speed):
    apparent_speed, apparent_direction_deg = apparent_wind(kayak_heading, kayak_speed, wind_direction, wind_speed)

    # Create a circular plot with compass headings
    plt.figure(figsize=(8, 8))
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)

    # Normalize speeds for plotting
    kayak_speed_normalized = kayak_speed / max(kayak_speed, wind_speed, apparent_speed)
    wind_speed_normalized = wind_speed / max(kayak_speed, wind_speed, apparent_speed)
    apparent_speed_normalized = apparent_speed / max(kayak_speed, wind_speed, apparent_speed)

    # Plot kayak's velocity
    kayak_heading_rad = math.radians(kayak_heading)
    kayak_velocity_x = kayak_speed_normalized * math.cos(kayak_heading_rad)
    kayak_velocity_y = kayak_speed_normalized * math.sin(kayak_heading_rad)
    kayak_arrow = ax.arrow(0, 0, kayak_velocity_x, kayak_velocity_y, head_width=0.1, head_length=0.1, fc='b', ec='b',
                           label="Kayak's Velocity\nMagnitude: {:.2f} km/h".format(kayak_speed))

    # Plot wind's velocity
    wind_direction_rad = math.radians(wind_direction)
    wind_velocity_x = wind_speed_normalized * math.cos(wind_direction_rad)
    wind_velocity_y = wind_speed_normalized * math.sin(wind_direction_rad)
    wind_arrow = ax.arrow(0, 0, wind_velocity_x, wind_velocity_y, head_width=0.1, head_length=0.1, fc='r', ec='r',
                          label="Wind's Velocity\nMagnitude: {:.2f} km/h".format(wind_speed))

    # Plot relative wind's velocity
    apparent_direction_rad = math.radians(apparent_direction_deg)
    apparent_velocity_x = apparent_speed_normalized * math.cos(apparent_direction_rad)
    apparent_velocity_y = apparent_speed_normalized * math.sin(apparent_direction_rad)
    apparent_arrow = ax.arrow(0, 0, apparent_velocity_x, apparent_velocity_y, head_width=0.1, head_length=0.1, fc='g', ec='g',
                              label="Relative Wind's Velocity\nMagnitude: {:.2f} km/h".format(apparent_speed))

    # Draw compass headings
    compass_headings = [0, 45, 90, 135, 180, 225, 270, 315]
    for heading in compass_headings:
        angle_rad = math.radians(heading)
        plt.text(1.3 * math.cos(angle_rad), 1.3 * math.sin(angle_rad), str(heading) + '°')

    plt.legend()
    plt.show()



# Example usage
kayak_heading = 0  # North-east direction
kayak_speed = 3  # km/h
wind_direction =  270 # South-east direction (opposite to kayak_heading)
wind_speed = 5  # knot







apparent_speed, apparent_direction = apparent_wind(kayak_heading, kayak_speed, wind_direction, wind_speed)

plot_apparent_wind(kayak_heading, kayak_speed, wind_direction, wind_speed)

# Print apparent magnitude of force on kayak
