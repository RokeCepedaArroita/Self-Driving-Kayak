class Weather:
    def __init__(self, wind_speed=0, wind_direction=180, current=0, current_direction=180):
        self.wind_speed = wind_speed # in km/h
        self.wind_direction = wind_direction # deg
        self.current = current # in km/h
        self.current_direction = current_direction # deg
        self.weathercocking_constant = 1 # in m^-3, determines the magnitude of torque exerted by the wind


    def apparent_wind(kayak_heading, kayak_speed, wind_heading, wind_speed):
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
        wind_heading_rad = np.deg2rad(wind_heading)

        # Convert speeds and headings to Cartesian coordinates
        kayak_vx = kayak_wind_speed * np.cos(kayak_heading_rad)
        kayak_vy = kayak_wind_speed * np.sin(kayak_heading_rad)
        wind_vx = wind_speed * np.cos(wind_heading_rad)
        wind_vy = wind_speed * np.sin(wind_heading_rad)

        # Calculate apparent wind speed and direction
        apparent_vx = wind_vx + kayak_vx
        apparent_vy = wind_vy + kayak_vy
        apparent_speed = np.sqrt(apparent_vx**2 + apparent_vy**2)
        apparent_angle = np.rad2deg(np.arctan2(apparent_vy, apparent_vx))

        # Calculate the apparent wind angle relative to the kayak
        apparent_angle_relative_to_kayak = apparent_angle - kayak_heading
        apparent_angle_relative_to_kayak %= 360

        # Return apparent wind speed and direction relative to the kayak
        return apparent_speed, apparent_angle


    def weathercocking(self, kayak_angle, kayak_speed, kayak_length):
        ''' Wind will exert a torque on a boat, making it stable only when it is perpendicular
            to the incoming wind. The magnitude of this effect needs to be experimentally
            measured by matching the time that it takes for a boat to turn from 45 deg relative
            angle to perpendicular to the wind and matching it to the proportionality below. If
            the turn-opposing effect of the water on the kayak is not accounted for, the
            correction will be an underestimate of the torque exerted by the wind, in part
            accounting for the water '''

        # Subtract the kayak motion from the wind motion to get the apparent wind speed and relative angle
        # relative to the frame of reference of the kayak


        # Relative angle of wind hitting the kayak
        relative_angle = shortest_angle_difference(kayak_angle, self.wind_direction+180) # deg
        # Calculate the torque caused by the wind on the kayak
        self.weathercocking_torque = self.weathercocking_constant * self.wind_speed**2 * (kayak_length/2) * np.sin( 2 * np.deg2rad(relative_angle))

        return self.weathercocking_torque


    # TODO: Calculate drag from water, dependent on velocity (get constant from terminal velocity at a fixed power level)

    def water_drag():
        ''' Simple model for the drag force of the water. Make it independent of angle for now '''

        return 0
