import math

def relative_wind(kayak_heading, kayak_speed, wind_direction, wind_speed):
    # Convert headings and wind directions from degrees to radians
    kayak_heading_rad = math.radians(kayak_heading)
    wind_direction_rad = math.radians(wind_direction)

    # Calculate the components of kayak's velocity and wind's velocity
    kayak_velocity_x = kayak_speed * math.cos(kayak_heading_rad)
    kayak_velocity_y = kayak_speed * math.sin(kayak_heading_rad)

    wind_velocity_x = wind_speed * math.cos(wind_direction_rad)
    wind_velocity_y = wind_speed * math.sin(wind_direction_rad)

    # Calculate the relative velocity components
    relative_velocity_x = wind_velocity_x - kayak_velocity_x
    relative_velocity_y = wind_velocity_y - kayak_velocity_y

    # Calculate the magnitude and direction of the relative velocity
    relative_speed = math.sqrt(relative_velocity_x**2 + relative_velocity_y**2)
    relative_direction_rad = math.atan2(relative_velocity_y, relative_velocity_x)

    # Convert the relative direction from radians to degrees
    relative_direction_deg = math.degrees(relative_direction_rad)

    # Ensure the relative direction is between 0 and 360 degrees
    relative_direction_deg = (relative_direction_deg + 360) % 360

    return relative_speed, relative_direction_deg


import math
import matplotlib.pyplot as plt


def plot_relative_wind(kayak_heading, kayak_speed, wind_direction, wind_speed):
    relative_speed, relative_direction_deg = relative_wind(kayak_heading, kayak_speed, wind_direction, wind_speed)

    # Create a circular plot with compass headings
    plt.figure(figsize=(8, 8))
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)

    # Normalize speeds for plotting
    kayak_speed_normalized = kayak_speed / max(kayak_speed, wind_speed, relative_speed)
    wind_speed_normalized = wind_speed / max(kayak_speed, wind_speed, relative_speed)
    relative_speed_normalized = relative_speed / max(kayak_speed, wind_speed, relative_speed)

    # Plot kayak's velocity
    kayak_heading_rad = math.radians(kayak_heading)
    kayak_velocity_x = kayak_speed_normalized * math.cos(kayak_heading_rad)
    kayak_velocity_y = kayak_speed_normalized * math.sin(kayak_heading_rad)
    kayak_arrow = ax.arrow(0, 0, kayak_velocity_x, kayak_velocity_y, head_width=0.1, head_length=0.1, fc='b', ec='b',
                           label="Kayak's Velocity\nMagnitude: {:.2f} knots".format(kayak_speed))

    # Plot wind's velocity
    wind_direction_rad = math.radians(wind_direction)
    wind_velocity_x = wind_speed_normalized * math.cos(wind_direction_rad)
    wind_velocity_y = wind_speed_normalized * math.sin(wind_direction_rad)
    wind_arrow = ax.arrow(0, 0, wind_velocity_x, wind_velocity_y, head_width=0.1, head_length=0.1, fc='r', ec='r',
                          label="Wind's Velocity\nMagnitude: {:.2f} knots".format(wind_speed))

    # Plot relative wind's velocity
    relative_direction_rad = math.radians(relative_direction_deg)
    relative_velocity_x = relative_speed_normalized * math.cos(relative_direction_rad)
    relative_velocity_y = relative_speed_normalized * math.sin(relative_direction_rad)
    relative_arrow = ax.arrow(0, 0, relative_velocity_x, relative_velocity_y, head_width=0.1, head_length=0.1, fc='g', ec='g',
                              label="Relative Wind's Velocity\nMagnitude: {:.2f} knots".format(relative_speed))

    # Draw compass headings
    compass_headings = [0, 45, 90, 135, 180, 225, 270, 315]
    for heading in compass_headings:
        angle_rad = math.radians(heading)
        plt.text(1.3 * math.cos(angle_rad), 1.3 * math.sin(angle_rad), str(heading) + '°')

    plt.legend()
    plt.show()



# Example usage
kayak_heading = 90  # North-east direction
kayak_speed = 5  # knots
wind_direction = 0  # South-east direction (opposite to kayak_heading)
wind_speed = 8  # knots

plot_relative_wind(kayak_heading, kayak_speed, wind_direction, wind_speed)


relative_speed, relative_direction = relative_wind(kayak_heading, kayak_speed, wind_direction, wind_speed)
print("Relative Wind Speed:", relative_speed, "knots")
print("Relative Wind Direction:", relative_direction, "degrees")
