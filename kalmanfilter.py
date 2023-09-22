import numpy as np

class KalmanFilter:
    def __init__(self, measurement_noise_covariance, alpha_fading_memory):

        from angular_tools import shortest_angle_difference

        # Initial state -- this should be set to the starting value
        self.x = np.zeros(2)  # [compass, angular_velocity]

        # Process Noise Covariance -- this is like a systematic process error due to sensor noise
        self.Q = np.diag([0.122 * 0.017**2, 0.017**2]) # [0, sigma_angular_velocity**2] is also close enough when dt is small

        # Measurement Covariance -- this is the error of your sensors
        self.R = measurement_noise_covariance

        # Identity matrix
        self.I = np.eye(2)

        # Initial State Covariance Matrix - Estimate Covariance - give the initial state no importance, since we initialize it with zeroes anyway
        self.P = np.diag([1000**2, 1000**2])

        # Fading memory term so that the filter tracks accelerations faster (less lag) in exchange for a little extra noise
        self.alpha = alpha_fading_memory


        self.shortest_angle_difference = shortest_angle_difference


    def predict(self, dt):

        # State transition matrix
        F = np.array([[1, dt],
                      [0, 1]])

        # Predicted (a priori) state estimate
        self.x = np.matmul(F, self.x)

        # Predicted (a priori) estimate covariance
        self.P = np.matmul(F, np.matmul(self.P, F.T)) + self.Q

        # Apply fading memory so that only a certain number of past points are effectively remembered
        self.P  = (self.alpha**2) * self.P


    def update(self, z):

        # Observation Matrix
        H = np.eye(2)

        # Innovation or measurement residual
        y = z - np.matmul(H, self.x)
        y[0] = self.shortest_angle_difference(self.x[0], z[0])  # Adjust the heading difference

        # System uncertainty (Innovation (or residual) covariance)
        S = np.matmul(H, np.matmul(self.P, H.T)) + self.R

        # Invert S
        S_inverse = np.linalg.inv(S);

        # Calculate Epsilon: the Mahalanobis distance of the measurement residual
        epsilon =  np.matmul(np.matmul(y.T, S_inverse), y);

        # Optimal Kalman gain
        K = np.matmul(np.matmul(self.P, H.T), np.linalg.inv(S))

        # Updated (a posteriori) state estimate
        self.x += np.matmul(K, y)

        # Updated (a posteriori) estimate covariance
        self.P = np.matmul(self.I - np.matmul(K, H), self.P)
