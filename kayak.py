import numpy as np

class Kayak:
    def __init__(self, length=3.3, width=0.9, mass=150, engine_spacing=0.25,
                 max_thrust=2.26, max_power=175, initial_angle=90, initial_angular_velocity=0,
                 initial_speed=0, weather=None):

        # Kayak parameters
        self.length  = length                # m, effective length
        self.width   = width                 # m, effective width
        self.mass    = mass                  # kg
        self.engine_spacing = engine_spacing # m from center of axis to each engine
        self.max_thrust = max_thrust         # kg, from one engine
        self.max_power  = max_power          # W,  from one engine

        # Forward drag and propulsion efficiency (experimentally measured)
        self.overall_efficiency = 0.206            # overall propulsion efficiency in converting power to forward push force (includes electrical, motor friction and propeller losses)
        self.CdA = 0.101*self.overall_efficiency   # m^2, effective drag_coefficient*effective_cross_sectional_area, standard deviation is 0.018 (18%). The uncorrected CdA constant corresponds to the force the motor would generate at 100% efficiency. However the propeller efficiency is lower due to propeller, motor and electrical losses.Therefore, the real-world CdA constant is given by CdA*efficiency.

        # Kinematics
        self.position = [0,0]                             # position in m
        self.speed = initial_speed                        # km/h
        self.acceleration = 0                             # m/s^2 (initial acceleration set to 0)
        self.angle = initial_angle                        # heading in degrees
        self.angular_velocity = initial_angular_velocity  # deg/s

        # Weather
        self.weather = weather     # weather class with parameters and functions

        # Calculate angular moment of inertia (maximum authority 2.3 deg/s^2 without reverse in my case)
        self.I = (1/12) * self.mass * (self.length**2 + self.width**2) # about 146 kg m^2


    def power_to_thrust(self, engine_power_level):
        ''' Converts the engine power (0-100%) into a thrust in kg,
            based on a fit to experimental data. 100% thrust is
            defined as 175W (2.26 kg) '''

            power = (engine_power_level/100)*self.max_power
            thrust = 0.135 * power**0.546

        return thrust # in kg


    def update(self, left_engine_power, right_engine_power, dt):
        ''' Update linear movement and rotation using Euler's method '''

        # Convert engine power from percentage to actual thrust in Newtons
        left_thrust  =  power_to_thrust(left_engine_power ) * 9.81 # left_engine_power  / 100 * self.max_thrust * 9.81
        right_thrust =  power_to_thrust(right_engine_power) * 9.81 # right_engine_power / 100 * self.max_thrust * 9.81

        # Drag force in Newtowns from water resistance
        water_drag = self.weather.water_drag()

        # Calculate linear acceleration in m/s^2
        # Note: this calculation simplifies the force as if it was applied to the center
        # of mass, which is a fairly small simplification if the engines are close to the center
        self.acceleration = (left_thrust+right_thrust-water_drag)/self.mass # m/s^2, maximum is 0.31 m/s^2 in my case

        # Update the speed in km/h
        self.speed = (self.speed/3.6 + self.acceleration*dt)*3.6

        # Update the position in space in meters
        self.position[0] += (self.speed/3.6)*np.cos(np.deg2rad(self.angle))*dt + 0.5*self.acceleration*np.cos(np.deg2rad(self.angle))*dt**2
        self.position[1] += (self.speed/3.6)*np.sin(np.deg2rad(self.angle))*dt + 0.5*self.acceleration*np.sin(np.deg2rad(self.angle))*dt**2


        # Calculate the engine torque, positive is to the right (clockwise if seen from above)
        engine_torque = (left_thrust - right_thrust) * self.engine_spacing

        # TODO: Add weather torque, positive is to the right (clockwise if seen from above)
        if self.weather is not None:
            wind_torque = self.weather.weathercocking(self.angle, self.speed, self.length)
        else:
            wind_torque = 0

        # Calculate the net torque, positive is to the right (clockwise if seen from above)
        net_torque = engine_torque + wind_torque

        # simple rotational friction
        friction = 0.25
        if net_torque > 0:
            net_torque += +friction*self.angular_velocity
        if net_torque < 0:
            net_torque += -friction*self.angular_velocity

        # Calculate angular acceleration in deg/s**2, positive is to the right (clockwise if seen from above)
        angular_acceleration = np.rad2deg(net_torque / self.I)

        # Update the angular velocity in deg/s
        self.angular_velocity += angular_acceleration*dt

        # Update the angle in deg
        self.angle += self.angular_velocity*dt + 0.5*angular_acceleration*dt**2

        # Return the new angle and angular velocity
        return self.angle, self.angular_velocity
