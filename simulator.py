import math
import numpy as np
import matplotlib.pyplot as plt



class Simulator:
    def __init__(self, timestep=0.122, simulated_time=60, kayak=None, weather=None, autopilot=None,
                 disturbance=None, sensor_noise=None, phase_delay=True, plot=False):

        # Simulation parameters
        self.timestep = timestep              # s, time interval at which physics and autopilot are simulated.
        self.simulated_time = simulated_time  # s, total duration to be simulated
        self.starting_time = 0                # s, starting time
        self.kayak       = kayak              # kayak object
        self.weather     = weather            # weather object
        self.autopilot   = autopilot          # autopilot object
        self.plot        = plot               # wether to plot the simulation or not (True/False)
        self.disturbance = disturbance        # dictionary that redefines wind parameters at a specified time in s (e.g. {'time': 10, 'wind_speed': 20, 'wind_heading': 180})
        self.phase_delay = phase_delay     # true/false: apply delay of one timestep between the time that the data is collected and the moment the autopilot receives and acts on that data

        # Simulation data
        self.data = {'time':                           [],
                     'angle':                          [],
                     'angular_velocity':               [],
                     'position_x':                     [],
                     'position_y':                     [],
                     'speed':                          [],
                     'acceleration':                   [],
                     'left_engine_power' :             [],
                     'right_engine_power':             [],
                     'apparent_wind_heading_absolute': [],
                     'target_heading':                 [],
                     'simulated_heading':              [],
                     'filtered_heading':               [],
                     'simulated_angular_speed':        [],
                     'filtered_angular_speed':         [],
                     'target_power':                   []}


        # Realistic sensor noise used to simulate sensor readings and determine the PID deadband
        if sensor_noise is None:
            self.sensor_noise = {'compass_heading': 1.04,   # deg, sensor sensitivity 1 standard deviation
                                 'angular_speed'  : 0.017,  # deg/s, sensor sensitivity 1 standard deviation
                                 'covariance'     : 0    }  # cov(compass, angular_speed) in deg^2/s
        else:
            self.sensor_noise = sensor_noise


    # Function to append simulation data
    def add_data(self, data_type, value):
        if data_type in self.data:
            self.data[data_type].append(value)
        else:
            raise ValueError(f"Invalid data type: {data_type}")


    # Create covariance matrix
    def create_covariance_matrix(self, sensor_noise):
        covariance_matrix = [ [ sensor_noise['compass_heading']**2, sensor_noise['covariance']      ],
                              [ sensor_noise['covariance']        , sensor_noise['angular_speed']**2] ]
        return covariance_matrix


    # Simulate sensors
    def simulate_sensors(self, heading, angular_speed, sensor_noise):
        ''' Simulate sensor noise '''

        # Build covariance matrix
        sensor_covariance = self.create_covariance_matrix(sensor_noise)

        # Build mean
        sensor_mean = [heading, angular_speed]

        def generate_correlated_data(mean, covariance, n_samples=1):
            ''' Generate correlated x,y data '''
            x, y = np.random.multivariate_normal(mean, covariance, n_samples).T
            if n_samples == 1:
                x = x[0]
                y = y[0]
            return x, y

        # Generate simulated sensor readouts
        heading, angular_speed = generate_correlated_data(sensor_mean, sensor_covariance)

        return heading, angular_speed

    # # Simulate sensors
    # def simulate_sensors(self, heading, angular_speed, sensor_noise):
    #     ''' Simulate sensor noise '''
    #
    #     # Simple noise simulation
    #     sensor_covariance = np.random.normal(heading, sensor_noise['compass_heading'])
    #     angular_speed     = np.random.normal(angular_speed, sensor_noise['compass_heading'])
    #
    #     return heading, angular_speed


    # Simulation function
    def simulate_kayak(self):
        ''' Run simulation of kayak '''

        # Start time
        time = self.starting_time  # starting time in s

        # Initialize previous data values, which will be stored to introduce a lag to the autopilot data
        simulated_heading_previous       = None
        simulated_angular_speed_previous = None

        # Simulate
        for step in np.arange(0, np.round(self.simulated_time / self.timestep)):

            # Simulate sensor noise
            simulated_heading, simulated_angular_speed = self.simulate_sensors(self.kayak.angle, self.kayak.angular_velocity, self.sensor_noise)

            # Control
            if self.phase_delay == False: # give current data (no phase delay) to the simulator
                left_engine_power, right_engine_power = self.autopilot.PIDController(simulated_heading, simulated_angular_speed)
            else:
                if simulated_heading_previous is None and simulated_angular_speed_previous is None:
                    left_engine_power, right_engine_power = 0, 0 # No control because we have no data
                else: # give one-timestep-old data to the autopilot
                    left_engine_power, right_engine_power = self.autopilot.PIDController(simulated_heading_previous, simulated_angular_speed_previous)

            # Effect
            angle, angular_velocity, position, speed, acceleration, apparent_wind_heading_absolute, position = \
                self.kayak.update(left_engine_power, right_engine_power, dt=self.timestep)

            # Advance time
            time = time + self.timestep

            # Apply wind disturbances as a sudden change in wind speed/direction
            if self.disturbance is not None:
                if time > self.disturbance['time']:
                    self.weather.wind_speed   = self.disturbance['wind_speed']
                    self.weather.wind_heading = self.disturbance['wind_heading']

            # Unpack positions
            position_x, position_y = position

            # Append the data
            if simulated_heading_previous is None and simulated_angular_speed_previous is None:
                autopilot_filtered_heading_value = np.nan
                autopilot_filtered_angular_velocity_value = np.nan
            else:
                autopilot_filtered_heading_value = self.autopilot.filtered_heading
                autopilot_filtered_angular_velocity_value = self.autopilot.filtered_angular_velocity

            data_to_add = [('time', time),
                ('angle', angle),
                ('angular_velocity', angular_velocity),
                ('position_x', position_x),
                ('position_y', position_y),
                ('speed', speed),
                ('acceleration', acceleration),
                ('left_engine_power', left_engine_power),
                ('right_engine_power', right_engine_power),
                ('apparent_wind_heading_absolute', apparent_wind_heading_absolute),
                ('target_heading', self.autopilot.target_heading),
                ('simulated_heading', simulated_heading),
                ('filtered_heading', autopilot_filtered_heading_value),
                ('simulated_angular_speed', simulated_angular_speed),
                ('filtered_angular_speed', autopilot_filtered_angular_velocity_value),
                ('target_power', self.autopilot.target_power)]
            for data_type, value in data_to_add:
                self.add_data(data_type, value)

            # Copy previous sensor values
            simulated_heading_previous = simulated_heading
            simulated_angular_speed_previous = simulated_angular_speed

            # if 451.5 > angle > 448.5:
            #     print(f'At {time:.1f} s I am at {angle:.2f} deg, moving at {angular_velocity:.1f} deg/s')
            # if 89.5 < angle < 90.5 and time < 30:
            #     print(f'At {time:.1f} s I am at {angle:.2f} deg, moving at {angular_velocity:.1f} deg/s')



        # Plot simulation
        if self.plot:
            self.plot_simulation()


        return self.data




    def plot_simulation(self):
        ''' Visualise simulation data '''

        # Overall lot
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(9, 9), gridspec_kw={'height_ratios': [1, 2, 1]})

        # Plot speed and acceleration on the first subplot
        maximum_speed = self.weather.maximum_speed(self.autopilot.target_power, self.kayak.power_to_thrust, self.kayak.CdA)
        ax1.plot(self.data['time'], self.data['speed'], color='C0', linewidth=3, label='Speed (km/h)')
        ax1.plot(self.data['time'], np.multiply(self.data['acceleration'],3.6*6), color='C3', linewidth=3, label='Acceleration (6*(km/h)/s)')
        ax1.plot(self.data['time'], np.multiply(self.data['simulated_angular_speed'],0.1), color='C3', label=r'Sensor $\omega$ (0.1*deg/s)', linewidth=2, alpha=0.2, zorder=1)
        ax1.plot(self.data['time'], np.multiply(self.data['filtered_angular_speed'],0.1), color='C2', label=r'Filtered $\omega$ (0.1*deg/s)', linewidth=2, alpha=0.2, zorder=1)
        ax1.axhline(y=maximum_speed, color='C0', linestyle='-', linewidth=10, alpha=0.15, label='Max Target Speed')
        ax1.axhline(y=0, color='k', linestyle='-', linewidth=1, alpha=0.15)
        ax1.axvline(x=10, color='k', linestyle='--', linewidth=2, alpha=0.1)
        ax1.set_ylabel('Speed and Acceleration')
        ax1.set_ylim([-2,7])
        ax1.set_xlim([0,np.max(self.data['time'])])
        ax1.set_title(fr'$Kp$={self.autopilot.kp}, $Kd$={self.autopilot.kd}, $Ki$={self.autopilot.ki}')
        ax1.legend()

        # Plot headings on the second subplot

        # Replace the None when autopilot is off for Nans
        self.data['target_heading'] = [val if val is not None else np.nan for val in self.data['target_heading']]
        heading_plot_ylimits = [np.min([np.min(self.data['angle'])-10, np.nanmin(self.data['target_heading'])]) , \
                                np.max([np.max(self.data['angle'])+10, np.nanmax(self.data['target_heading'])])]

        ax2.plot(self.data['time'], self.data['angle'], label='Kayak', linewidth=5, zorder=0)
        ax2.plot(self.data['time'], self.data['simulated_heading'], color='C3', label='Sensor Data', linewidth=2, alpha=0.2, zorder=1)
        ax2.plot(self.data['time'], self.data['filtered_heading'], color='C2', label='Filtered Data', linewidth=2, alpha=0.2, zorder=1)
        ax2.axvline(x=10, color='k', linestyle='--', linewidth=2, alpha=0.1)
        ax2.set_ylabel('Heading (deg)')
        for termination in [0,360,-360]:
            ax2.axhline(y=termination, color='k', linestyle='-', linewidth=1, alpha=0.8)
        ax2.set_ylim(heading_plot_ylimits)

        for target_points in [self.data['target_heading'], np.add(self.data['target_heading'],360), np.add(self.data['target_heading'],-360)]:
            if any(heading_plot_ylimits[0] <= point <= heading_plot_ylimits[1] for point in target_points): # only plot if the apparent wind attactor is within the plot limits
                ax2.plot(self.data['time'], target_points, color='g', linestyle='--', linewidth=3, label='Target')


        # If there are any wind stable points within the plot, draw them
        if self.weather.wind_speed != 0:
            stable_wind_points = [(self.weather.wind_heading + 90) % 360, (self.weather.wind_heading - 90) % 360, \
                                  (self.weather.wind_heading + 90) % 360 - 360, (self.weather.wind_heading - 90) % 360 + 360]
            apparent_stable_wind_points = [np.mod(np.add(self.data['apparent_wind_heading_absolute'], +90), 360), \
                                           np.mod(np.add(self.data['apparent_wind_heading_absolute'], -90), 360), \
                                           np.mod(np.add(self.data['apparent_wind_heading_absolute'], +90), 360)-360,\
                                           np.mod(np.add(self.data['apparent_wind_heading_absolute'], -90), 360)+360]
            for stable_wind_point in stable_wind_points:
                if heading_plot_ylimits[0] < stable_wind_point < heading_plot_ylimits[1]:
                    ax2.axhline(y=stable_wind_point, color='r', linewidth=10, alpha=0.1, linestyle='-', label='Wind attractor')
            for wind_point_dir in apparent_stable_wind_points:
                if any(heading_plot_ylimits[0] <= point <= heading_plot_ylimits[1] for point in wind_point_dir): # only plot if the apparent wind attactor is within the plot limits
                    ax2.plot(self.data['time'], wind_point_dir, color='b', linewidth=10, alpha=0.1, linestyle='-', label='Apparent attractor')



        # Set the custom y-axis tick labels
        from angular_tools import wrap_angles
        from matplotlib.ticker import FuncFormatter
        #ax2.yaxis.set_major_formatter(FuncFormatter(wrap_angles))

        ax2.legend(loc='upper right')


        # Plot engine powers on the third subplot
        combined_engine_power = np.add(self.data['right_engine_power'],self.data['left_engine_power'])
        ax3.plot(self.data['time'], self.data['right_engine_power'], color='r', label='Right', zorder=2)
        ax3.plot(self.data['time'], self.data['left_engine_power'],  color='b', label='Left', zorder=2)
        plt.fill_between(self.data['time'], combined_engine_power, 0, alpha=0.1, color='black', label='Combined', zorder=1)
        ax3.plot(self.data['time'], self.data['target_power'], linestyle=':', alpha=0.3, color='black', label='Target', zorder=0)
        ax3.axvline(x=10, color='k', linestyle='--', linewidth=2, alpha=0.1)
        ax3.set_ylabel('Engine Power (%)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylim([-1,101])
        ax3.set_xlim([0,np.max(self.data['time'])])
        ax3.legend()
        plt.tight_layout()

        plt.show()


        return





# # Example simulation below
#
# from kayak import Kayak
# from weather import Weather
# from autopilot import Autopilot
#
# # Initialize weather: wind limit 28 km/h
# weather = Weather(wind_speed=17, wind_heading=180)
#
# # Initialize kayak
# kayak = Kayak(initial_speed=0, initial_heading=45, weather=weather)
#
# # Initialize autopilot (Pessen: kp=49, kd=68, ki=13, Classic: kp=42, kd=49, ki=9, No overshoot: kp=14, kd=43, ki=3, Some Overshoot: kp=23, kd=72, ki=5)
# autopilot = Autopilot(kp=49, kd=68, ki=13, smoothing_time=0.1, derivative_smoothing_time=0.4, target_power=25, target_heading=135)
#
# # Initialize simulator
# simulator = Simulator(simulated_time=100, kayak=kayak, weather=weather, autopilot=autopilot, plot=True)
#
# data = simulator.simulate_kayak()


#
#
#
# # Animate position below
#
# from matplotlib.animation import FuncAnimation
#
# def update_plot(frame):
#     # Choose the update interval (e.g., update every 5 frames)
#     update_interval = 15
#     sc = None
#     # Only update the plot when the frame is a multiple of the update_interval
#     if frame % update_interval == 0 or frame == len(data['time']) - 1:
#         # Clear the previous scatter object if it exists
#         if sc is not None:
#             sc.remove()
#         # Get the colormap and norm
#         cmap = plt.get_cmap('viridis')
#         norm = plt.Normalize(np.min(data['speed']), np.max(data['speed']))
#         # Get the color for each point based on speed
#         color = cmap(norm(data['speed'][frame]))
#         # Use scatter to plot the data with color based on speed
#         sc = ax.plot(data['position_x'][frame], data['position_y'][frame], linestyle='', marker='o', color=color)
#         # sc = ax.scatter(data['position_x'][frame + 1], data['position_y'][frame + 1], color=color, marker='o')
#         # Timer text
#         timer_text.set_text('Time: {:.2f} seconds'.format(data['time'][frame]))
#
#     return sc  # Return the scatter object to be used by the colorbar
#
#
#
# # Create a figure
# fig, ax = plt.subplots(figsize=(5, 4))  # Adjust width and height as needed
#
#
# # Set axis limits (adjust according to your data range)
# ax.set_xlim([np.min(data['position_x'])-2, np.max(data['position_x'])+2])
# ax.set_ylim([np.min(data['position_y'])-2, np.max(data['position_y'])+2])
# ax.set_ylabel('South-North (m)')
# ax.set_xlabel('West-East (m)')
# ax.set_aspect('equal')
# ax.grid(True)
#
# # Add the timer text
# timer_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12, color='black')
#
# # Create the animation
# animation = FuncAnimation(fig, update_plot, frames=len(data['time']), interval=5, repeat=False)
#
# # Get the colormap and norm
# cmap = plt.get_cmap('viridis')
# norm = plt.Normalize(np.min(data['speed']), np.max(data['speed']))
#
# # Create a ScalarMappable with the colormap and norm
# sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
#
# # Set the data for the ScalarMappable, but we only need the colorbar, so an arbitrary data point is sufficient
# sm.set_array(data['speed'])
#
# # Add a colorbar next to the plot
# cax = plt.axes([0.92, 0.1, 0.02, 0.8])  # [left, bottom, width, height]
# cbar = plt.colorbar(sm, cax=cax)  # Use the ScalarMappable for the colorbar
# cbar.set_label('Speed (km/h)')
#
# # To save the animation as a video file (optional)
# animation.save('position_animation.gif', writer='pillow')
#
# # Show the animation
# plt.show()
#
#
#
#
#
#
#
#
# # Fastest time for 90 deg turn is 8.80 s
#
# # KP 10, KD 85
