import numpy as np

class KalmanFilter:
    def __init__(self, sigma_compass, sigma_angular_velocity, dt):

        from angular_tools import shortest_angle_difference

        self.dt = dt

        # Initial state -- this should be set to the starting value
        self.x = np.zeros(2)  # [compass, angular_velocity]

        # Process noise covariance -- this is like a systematic process error due to sensor noise
        self.Q = np.diag([(sigma_angular_velocity*self.dt)**2, sigma_angular_velocity**2]) # [0, sigma_angular_velocity**2] is also close enough when dt is small

        # Measurement noise covariance -- this is the error of your sensors
        self.R = np.diag([sigma_compass**2, sigma_angular_velocity**2])

        # Identity matrix
        self.I = np.eye(2)

        # Initial State Covariance Matrix - give the initial state no importance, since we initialize it with zeroes anyway
        self.P = np.diag([1000*sigma_compass**2, 1000*sigma_angular_velocity**2])


        self.shortest_angle_difference = shortest_angle_difference

    def predict(self):
        # State transition matrix
        F = np.array([[1, self.dt],
                      [0, 1      ]])

        # Predicted (a priori) state estimate
        self.x = np.matmul(F, self.x)

        # Predicted (a priori) estimate covariance
        self.P = np.matmul(F, np.matmul(self.P, F.T)) + self.Q #np.matmul(np.matmul(F, self.P), F.T) + self.Q #

    def update(self, z):
        # Measurement matrix
        H = np.eye(2)

        # Innovation or measurement residual
        y = z - np.matmul(H, self.x)
        y[0] = self.shortest_angle_difference(self.x[0], z[0])  # Adjust the heading difference

        # Innovation (or residual) covariance
        S = np.matmul(H, np.matmul(self.P, H.T)) + self.R

        # Optimal Kalman gain
        K = np.matmul(np.matmul(self.P, H.T), np.linalg.inv(S))

        # Updated (a posteriori) state estimate
        self.x += np.matmul(K, y)

        # Updated (a posteriori) estimate covariance
        self.P = np.matmul(self.I - np.matmul(K, H), self.P)
