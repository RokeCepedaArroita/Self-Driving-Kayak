import numpy as np

class KalmanFilter:
    def __init__(self, sigma_compass, sigma_angular_velocity):
        from angular_tools import shortest_angle_difference

        self.x = np.zeros(2)  # [compass, angular_velocity]

        self.Q_compass = (sigma_angular_velocity * 0.08) ** 2
        self.Q_angular_velocity = sigma_angular_velocity ** 2

        self.R_compass = sigma_compass ** 2
        self.R_angular_velocity = sigma_angular_velocity ** 2

        # Do not trust the initial guess
        self.P_compass = 1000 * self.R_compass
        self.P_angular_velocity = 1000 * self.R_angular_velocity

        self.shortest_angle_difference = shortest_angle_difference

    def predict(self, dt):

        F_angular_velocity = dt

        self.x[0] = self.x[0] + F_angular_velocity * self.x[1]
        self.x[1] = self.x[1]

        self.P_compass = self.P_compass + self.Q_compass
        self.P_angular_velocity = F_angular_velocity * self.P_angular_velocity * F_angular_velocity + self.Q_angular_velocity

    def update(self, z):

        y_compass = z[0] - self.x[0]
        y_angular_velocity = z[1] - self.x[1]
        y_compass = self.shortest_angle_difference(self.x[0], z[0])

        S_compass = self.P_compass + self.R_compass
        S_angular_velocity = self.P_angular_velocity + self.R_angular_velocity

        K_compass = self.P_compass / S_compass
        K_angular_velocity = self.P_angular_velocity / S_angular_velocity

        self.x[0] = self.x[0] + K_compass * y_compass
        self.x[1] = self.x[1] + K_angular_velocity * y_angular_velocity

        self.P_compass = (1 - K_compass) * self.P_compass
        self.P_angular_velocity = (1 - K_angular_velocity) * self.P_angular_velocity
