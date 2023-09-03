import numpy as np

class Kayak:
    def __init__(self, length=3.46, effective_length=2.6, effective_width=0.7, mass=155,
                 center_of_mass=-0.141, engine_spacing=0.178, max_thrust=2.26, max_power=175,
                 min_power=2, initial_heading=90, initial_angular_velocity=0, initial_speed=0, weather=None):

        # Kayak parameters
        self.length  = length                      # m, total length
        self.effective_length = effective_length   # m, effective length for angular moment of inertia calculations
        self.width   = effective_width             # m, effective width for angular moment of inertia calculations
        self.mass    = mass                        # kg, total mass including kayak, passengers and cargo
        self.center_of_mass = center_of_mass       # m, offset between the geometric center and the center of mass (negative is towards the stern)
        self.engine_spacing = engine_spacing       # m from center of axis to each engine, 31.5 cm between skegs and 2.1 cm additional if engines are placed outside
        self.max_thrust = max_thrust               # kg, from one engine
        self.max_power  = max_power                # W, from one engine
        self.minimum_power = min_power             # W, power below which there is no thrust

        # Forward drag and propulsion efficiency characteristics (experimentally measured)
        # Note: The uncorrected CdA constant corresponds to the force the motor would generate
        # at 100% efficiency. However, the propeller efficiency is lower due to propeller,
        # motor and electrical losses.Therefore, the real-world CdA constant is given by
        # CdA*efficiency. A multiplier to the drag is added to account for the drag added
        # by the second motor, since the experimental numbers come from a single engine test.
        # The overall uncertainty of these numbers is around 30%.
        self.overall_efficiency = 0.206            # overall propulsion efficiency in converting power to forward push force (includes electrical, motor friction and propeller losses)
        self.twin_engine_drag_penalty = 1.113      # drag penalty incurred by placing a second engine
        self.CdA = 0.101*self.overall_efficiency*self.twin_engine_drag_penalty  # m^2, effective drag_coefficient*effective_cross_sectional_area, standard deviation is 18%
        self.turning_drag = 0.026 # Nm/(deg/s)^2, opposing torque proportional to angular_speed^2 caused primarily by back skegs hitting the water at an angle, experimentally calibrated (does not take speed into account)

        # Kinematics
        self.position = [0,0]                             # position in m
        self.speed = initial_speed                        # km/h
        self.acceleration = 0                             # m/s^2 (initial acceleration set to 0)
        self.angle = initial_heading                        # heading in degrees
        self.angular_velocity = initial_angular_velocity  # deg/s

        # Weather
        self.weather = weather     # weather class with parameters and functions

        # Calculate angular moment of inertia (maximum authority ~3 deg/s^2 without reverse in my case)
        self.I = (1/12) * self.mass * (self.effective_length**2 + self.width**2) # about 90 kg m^2


    def power_to_thrust(self, engine_power_level):
        ''' Converts the engine power (0-100%) into a thrust in kg, based on a fit
            to experimental data. 100% thrust is defined as 175W (2.26 kg), in the
            self.max_power variable. A constant efficiency propeller will have a
            thrust proportional to power^(2/3). A real propeller will typically have
            a lower index (~0.54) at high power, indicating worse efficiency at high
            power. To account this, the relation is modelled as a smoothly broken
            power law that starts with an index of 2/3 at low powers and transitions
            to an index of 0.54 above 70W '''

        power = (engine_power_level/100)*self.max_power
        thrust = 0.08456613727390482 * power**(2/3) * (1+power/70)**(-0.12806666666666666)

        # At very low values, the motor won't spin so it won't create thrust
        if power < self.minimum_power:
            thrust = 0

        return thrust # in kg


    def turning_drag_torque(self, net_torque):
        ''' Simple model that simulates the opposition to turning produced by the skegs.
            net_torque is the combined torque of the motors and wind in Nm, the angular
            velocity is in deg/s and turning_drag is also returned in Nm. More generally,
            the drag could be modelled as (D + C v + E alpha) omega**2 where v is the
            linear speed and alpha is the fixed angle of attack, and E-D-C are constants
            The omega**2 relationship comes from sin^2(x) ≈ x^2 at low x,
            where x is the angle of attack '''

        # Turning drag always has a sign opposite to angular velocity
        turning_drag = -self.turning_drag*np.sign(self.angular_velocity)*(self.angular_velocity)**2

        return turning_drag


    def update(self, left_engine_power, right_engine_power, dt):
        ''' Update linear movement and rotation using Euler's method '''

        # Convert engine power from percentage to actual thrust in Newtons
        left_thrust  =  self.power_to_thrust(left_engine_power ) * 9.81
        right_thrust =  self.power_to_thrust(right_engine_power) * 9.81

        # Drag force in Newtowns from water resistance
        water_drag = self.weather.water_drag(self.speed, self.CdA)

        # Calculate linear acceleration in m/s^2
        # Note: this calculation simplifies the force as if it was applied to the center
        # of mass, which is a fairly small simplification if the engines are close to the center
        self.acceleration = (left_thrust+right_thrust-water_drag)/self.mass # m/s^2, maximum is 0.31 m/s^2 in my case

        # Update the speed in km/h
        self.speed = (self.speed/3.6 + self.acceleration*dt)*3.6

        # Update the position in space in meters
        self.position[1] += (self.speed/3.6)*np.cos(np.deg2rad(self.angle))*dt + 0.5*self.acceleration*np.cos(np.deg2rad(self.angle))*dt**2
        self.position[0] += (self.speed/3.6)*np.sin(np.deg2rad(self.angle))*dt + 0.5*self.acceleration*np.sin(np.deg2rad(self.angle))*dt**2

        # Calculate the engine torque, positive is to the right (clockwise if seen from above)
        engine_torque = (left_thrust - right_thrust) * self.engine_spacing

        # Realistic wind torque (a.k.a weathercocking), positive is to the right (clockwise if seen from above)
        if self.weather is not None:
            # Set apparent wind speeds and directions due to the kayak motion
            self.weather.apparent_wind_speed, self.weather.apparent_wind_heading_relative, \
                self.weather.apparent_wind_heading_absolute = self.weather.apparent_wind(self.angle, self.speed)
            # Calculate torques and
            wind_torque = self.weather.weathercocking(self.angle, self.speed, self.length)
        else: # if the weather is not defined
            wind_torque = 0
            apparent_wind_heading_absolute = None

        # Calculate the net torque, positive is to the right (clockwise if seen from above)
        net_torque = engine_torque + wind_torque

        # Apply simple rotational friction caused by the back skegs
        net_torque += self.turning_drag_torque(net_torque)

        # Calculate the net angular acceleration in deg/s**2, positive is to the right (clockwise if seen from above)
        angular_acceleration = np.rad2deg(net_torque / self.I)

        # Update the angular velocity in deg/s
        self.angular_velocity += angular_acceleration*dt

        # Update the angle in deg
        self.angle += self.angular_velocity*dt + 0.5*angular_acceleration*dt**2

        # Return the new angle and angular velocity
        return self.angle, self.angular_velocity, self.position, self.speed, self.acceleration, \
               self.weather.apparent_wind_heading_absolute, self.position
