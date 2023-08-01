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
