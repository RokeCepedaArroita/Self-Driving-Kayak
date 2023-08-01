from turn_simulator import *

def optimize_pid_gains(desired_heading):

    # Define the parameter grid
    kp_range = np.linspace(1.55, 1.55, num=1) # proportional term
    ki_range = np.linspace(0, 0.0003, num=5) # integral term, not needed
    kd_range = np.linspace(0.5, 0.5, num=1) # derivative term

    # Initialize variables to store the best gains and best performance
    best_kp, best_ki, best_kd = None, None, None
    best_performance = float('inf')  # Initialize to a large value for minimization tasks

    # Grid search
    for kp in kp_range:
        for ki in ki_range:
            for kd in kd_range:
                # Simulate the PID controller with current gains and evaluate performance
                # For example, you can use a simulation or run actual experiments here
                current_performance = evaluate_performance(kp, ki, kd, desired_heading)

                # Update the best gains if the current performance is better
                if current_performance < best_performance:
                    best_performance = current_performance
                    best_kp, best_ki, best_kd = kp, ki, kd

    return best_kp, best_ki, best_kd

# Replace this function with your performance evaluation function based on the system's response
def evaluate_performance(kp, ki, kd, desired_heading):
    kayak = Kayak()
    times, angles, angular_velocities, left_engine_powers, right_engine_powers = simulate_heading_control(kp, ki, kd, kayak, desired_heading, plot=True)
    # Implement your performance evaluation function here
    # For example, you can use a simulation or run actual experiments to obtain a performance metric
    # based on the system's response to the PID controller with the given gains.
    performance_metric = np.max(np.abs(( shortest_angle_difference(desired_heading,angles[1000:])))) # np.abs( desired_heading - np.mean(angles[150:]) ) + np.std(np.abs( np.subtract(desired_heading,angles[150:]) ))

    print(f'Kp={kp}, Kd={kd}, Ki={ki}  ----> Performance {performance_metric}')

    return  performance_metric # Return the performance metric (lower values indicate better performance)

# Example usage:
best_kp, best_ki, best_kd = optimize_pid_gains(desired_heading)
print("Best kp:", best_kp)
print("Best ki:", best_ki)
print("Best kd:", best_kd)
