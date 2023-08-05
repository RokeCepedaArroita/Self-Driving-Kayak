import numpy as np

def shortest_angle_difference(current_heading, desired_heading):
    # Convert inputs to numpy arrays for handling lists
    current_heading = np.asarray(current_heading)
    desired_heading = np.asarray(desired_heading)

    # Ensure both angles are within the range [0, 360)
    current_heading %= 360
    desired_heading %= 360

    # Calculate the raw difference between the angles
    raw_difference = np.subtract(desired_heading, current_heading)

    # Adjust the raw difference to the shortest angle (considering the wrap-around)
    shortest_difference = np.where(
        np.abs(raw_difference) <= 180,  # Condition for the shortest angle
        raw_difference,                 # If true, return raw_difference
        np.where(
            raw_difference > 180,       # Condition for turning left (negative)
            np.subtract(raw_difference, 360),  # If true, return raw_difference - 360
            np.add(raw_difference, 360)       # If false, return raw_difference + 360
        )
    )

    return shortest_difference


def compass_average(readings):
    # Convert compass headings to radians
    rad_readings = np.radians(readings)
    # Convert radians to 2D unit vectors
    vectors = np.array([(np.cos(rad), np.sin(rad)) for rad in rad_readings])
    # Calculate the mean of the vectors
    mean_vector = np.mean(vectors, axis=0)
    # Convert mean vector back to compass heading in degrees
    average_heading = np.degrees(np.arctan2(mean_vector[1], mean_vector[0]))
    if average_heading < 0:
        average_heading += 360
    return average_heading


def compass_stddev(readings):
    # Convert compass headings to radians
    rad_readings = np.radians(readings)
    # Convert radians to 2D unit vectors
    vectors = np.array([(np.cos(rad), np.sin(rad)) for rad in rad_readings])
    # Calculate the mean of the vectors
    mean_vector = np.mean(vectors, axis=0)
    # Calculate the circular variance
    circular_variance = 1 - np.linalg.norm(mean_vector)
    # Calculate the circular standard deviation
    circular_std_deviation = np.sqrt(-2 * np.log(1 - circular_variance))
    # Convert back to degrees
    circular_std_deviation = np.rad2deg(circular_std_deviation)
    return circular_std_deviation


def wrap_angles(angle, pos):
    ''' Wrap angles to format axis labels correctly '''
    wrapped_angle = int(np.round((angle) % 360))
    if wrapped_angle == 0:
        return '0'
    else:
        return str(wrapped_angle)
