import numpy as np

class Kayak:
    def __init__(self, length=3.3, width=0.9, mass=150, engine_spacing=0.25,
                 max_thrust=2.4, initial_angle=90, initial_angular_velocity=0,
                 initial_speed=4, weather=None):

        # Kayak parameters
        self.length  = length                # m, effective length
        self.width   = width                 # m, effective width
        self.mass    = mass                  # kg
        self.engine_spacing = engine_spacing # m from center of axis to each engine
        self.max_thrust = max_thrust         # kg, from one engine

        # Kinematics
        self.position = [0,0]       # position in m
        self.angle = initial_angle  # heading in degrees
        self.angular_velocity = initial_angular_velocity # deg/s
        self.speed = initial_speed # km/h
        self.acceleration = 0      # m/s^2 (initial acceleration set to 0)

        # Weather
        self.weather = weather     # weather class with parameters and functions

        # Calculate angular moment of inertia (maximum authority 2.3 deg/s^2 without reverse in my case)
        self.I = (1/12) * self.mass * (self.length**2 + self.width**2) # about 146 kg m^2


    def update(self, left_engine_power, right_engine_power, dt):
        ''' This only accounts for rotation, not acceleration '''

        # Convert engine power from percentage to actual thrust in Newtons
        left_thrust  = left_engine_power  / 100 * self.max_thrust * 9.81
        right_thrust = right_engine_power / 100 * self.max_thrust * 9.81

        # Drag force in Newtown from water resistance
        water_drag = self.weather.water_drag()

        # Calculate linear acceleration in m/s^2
        # Note: this calculation simplifies the force as if it was applied to the center of mass, which is a small simplification
        self.acceleration = (left_thrust+right_thrust-water_drag)/self.mass # m/s^2, maximum is 0.31 m/s^2 in my case

        # Update the speed in km/h
        self.speed = (self.speed/3.6 + self.acceleration*dt)*3.6

        # Update the position in space in meters
        self.position[0] += (self.speed/3.6)*np.cos(np.deg2rad(self.angle))*dt + 0.5*self.acceleration*np.cos(np.deg2rad(self.angle))*dt**2
        self.position[1] += (self.speed/3.6)*np.sin(np.deg2rad(self.angle))*dt + 0.5*self.acceleration*np.sin(np.deg2rad(self.angle))*dt**2



        # Calculate the net torque, positive is clockwise
        net_torque = (left_thrust - right_thrust) * self.engine_spacing 

        # TODO: Add weather torque, if any
        if self.weather is not None:
            wind_torque = weathercocking(weather, self.angle, self.speed, self.length)
            wind_angular_acceleration = 0

        # Calculate angular acceleration in deg/s**2, positive is to the rights
        angular_acceleration = np.rad2deg(net_torque / self.I)

        # Update the angular velocity in deg/s
        self.angular_velocity += angular_acceleration*dt

        # # Get average angular velocity during this step in deg/s -- it's close enough
        # average_angular_velocity = self.angular_velocity - angular_acceleration*dt/2

        # Update the angle in deg, use the average
        self.angle += average_angular_velocity*dt + 0.5*angular_acceleration*dt**2

        # Return the new angle and angular velocity
        return self.angle, self.angular_velocity
