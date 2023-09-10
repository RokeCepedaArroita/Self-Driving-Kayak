import numpy as np
from angular_tools import shortest_angle_difference

class Weather:
    def __init__(self, wind_speed=0, wind_heading=0, current=0, current_direction=0):
        self.wind_speed = wind_speed # in km/h, control maximum is about 28 km/h
        self.wind_heading = wind_heading # deg
        self.current = current # in km/h
        self.current_direction = current_direction # deg
        self.weathercocking_constant = 0.0037 # in m^-3, determines the magnitude of torque exerted by the wind
        self.water_density = 1010.5  # kg/m^3, 997 for fresh water, 1025 for sea water

        # Parameters that depend on kayak motion
        self.apparent_wind_heading_relative = None  # incoming angle relative to nose of kayak (+ve incoming from the right, and -ve from the left)
        self.apparent_wind_heading_absolute = None # same as above but as an absolute compass heading
        self.apparent_wind_speed   = None  # apparent wind speed in the kayak frame of reference as a result of the combined kayak and wind motion



    def apparent_wind(self, kayak_heading, kayak_speed):
        ''' Calculate the apparent wind felt in the kayak as a result of the combined motion of
            the kayak and the wind. Output the apparent wind speed and give the direction in
            relative terms to the nose of the kayak:

            0   deg  -> head-on wind
            90  deg  -> right to left
            180 deg  -> full tailwind
            270 deg  -> left to right

            '''

        # The apparent wind caused by the kayak's movement is the same as turning the kayak vector 180 deg
        # (e.g. as if the kayak was stationary and the wind hit it in the opposite direction). We can achieve
        # this in two ways, either by rotating 180 deg or by setting the magnitude to the opposite sign
        kayak_wind_speed = -kayak_speed # wind caused by kayak movement acts in the opposite direction

        # Convert headings to radians
        kayak_heading_rad = np.deg2rad(kayak_heading)
        wind_heading_rad = np.deg2rad(self.wind_heading)

        # Convert speeds and headings to Cartesian coordinates
        kayak_vx = kayak_wind_speed * np.cos(kayak_heading_rad)
        kayak_vy = kayak_wind_speed * np.sin(kayak_heading_rad)
        wind_vx = self.wind_speed * np.cos(wind_heading_rad)
        wind_vy = self.wind_speed * np.sin(wind_heading_rad)

        # Calculate apparent wind speed and direction
        apparent_vx = wind_vx + kayak_vx
        apparent_vy = wind_vy + kayak_vy
        apparent_speed = np.sqrt(apparent_vx**2 + apparent_vy**2)
        apparent_angle = np.rad2deg(np.arctan2(apparent_vy, apparent_vx))

        # Apparent angle relative to North-South-East-West
        apparent_angle_absolute = apparent_angle

        # Calculate the apparent wind angle relative to the kayak
        apparent_angle_relative_to_kayak = apparent_angle - kayak_heading
        apparent_angle_relative_to_kayak %= 360

        # Return apparent wind speed and direction relative to the kayak
        return apparent_speed, apparent_angle_relative_to_kayak, apparent_angle_absolute


    def wind_drag():
        ''' TODO: Implement linear drag caused by wind. Need experimental tests to calibrate this
            since the magnitude of the force depends on the orientation of the kayak. '''
        return


    def weathercocking(self, kayak_heading, kayak_speed, kayak_length):
        ''' Wind will exert a torque on a boat, making it stable only when it is perpendicular
            to the incoming wind. The magnitude of this effect needs to be experimentally
            measured by matching the time that it takes for a boat to turn from 45 deg relative
            angle to perpendicular to the wind and matching it to the proportionality below. If
            the turn-opposing effect of the water on the kayak is not accounted for, the
            correction will be an underestimate of the torque exerted by the wind, in part
            accounting for the water '''

        # Relative angle of wind hitting the kayak from the frame of reference of the kayak
        apparent_speed, apparent_angle, apparent_angle_absolute = self.apparent_wind(kayak_heading, kayak_speed) # km/h and deg

        # Calculate the torque caused by the wind on the kayak:
        # In the definition of apparent angle, if positive, it should turn the kayak to the left,
        # and if negative, it should turn it to the right. This is the opposite as the definition of
        # torque in the initial kayak setup. Therefore we add a minus sign (e.g. positive torque turns to the right)
        self.weathercocking_torque = - self.weathercocking_constant * apparent_speed**2 * (kayak_length/2) * np.sin( 2 * np.deg2rad(apparent_angle))

        return self.weathercocking_torque



    def water_drag(self, kayak_speed, CdA):
        ''' Simple model for the drag force of the water in Newtons when travelling at kayak_speed km/h.
            CdA is the effective drag_coefficient times the effective cross sectional area, which is
            measured experimentally. Note that the value of both A and Cd changes with different orientations
            (e.g. a side-on kayak will have a higher drag coefficient), and thus this value cannot be
            extrapolated to different orientations without real-world testing. '''

        drag_force = 0.5 * self.water_density * (kayak_speed/3.6)**2 * CdA # Newtons, input speed needs to be in km/h

        return drag_force # Newtons


    def maximum_speed(self, target_power_level, power_to_thrust, CdA):
        ''' The terminal speed achieved by the kayak under equal power levels in each engine
            The power_to_thrust function is one that will convert the engine power to thrust
            in kg, and one that takes the kayak object and the engine power level (0-100% as
            the input). CdA is the effective drag constant of the kayak in m^2'''

        engine_power_level = target_power_level/2. # half the power to each engine

        # Calculate thrust and convert to Newtons
        thrust = power_to_thrust(engine_power_level)*9.81
        thrust = thrust*2 # two engines in total

        # Calculate the speed at which both forces are equal
        terminal_speed = ((2*thrust)/(self.water_density*CdA))**(1/2) # in m/s
        terminal_speed = terminal_speed*3.6 # convert to km/h

        return terminal_speed # km/h
