import math
import numpy as np
import matplotlib.pyplot as plt



class Simulator:
    def __init__(self, timestep=0.08, simulated_time=60, kayak=None, weather=None, autopilot=None, plot=False):

        # Simulation parameters
        self.timestep = timestep              # s, time interval at which physics and autopilot are simulated.
        self.simulated_time = simulated_time  # s, total duration to be simulated
        self.starting_time = 0                # s, starting time
        self.kayak     = kayak                # kayak object
        self.weather   = weather              # weather object
        self.autopilot = autopilot            # autopilot object
        self.plot      = plot                 # wether to plot the simulation or not (True/False)

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
                     'target_heading':                 []}


        # TODO: Realistic sensor noise used to simulate sensor readings and determine the PID deadband
        self.sensor_noise = {'compass_heading': 1.04,   # deg, 1 standard deviation
                             'angular_speed'  : 0.017}  # deg/s, 1 standard deviation



    # Function to append simulation data
    def add_data(self, data_type, value):
        if data_type in self.data:
            self.data[data_type].append(value)
        else:
            raise ValueError(f"Invalid data type: {data_type}")


    # Simulation function
    def simulate_kayak(self):
        ''' Run simulation of kayak '''

        # Start time
        time = self.starting_time  # starting time in s

        # Simulate
        for step in np.arange(0, np.round(self.simulated_time / self.timestep)):

            # Control
            left_engine_power, right_engine_power = self.autopilot.PIDController(self.kayak.angle, self.kayak.angular_velocity)

            # Effect
            angle, angular_velocity, position, speed, acceleration, apparent_wind_heading_absolute, position = \
                self.kayak.update(left_engine_power, right_engine_power, dt=self.timestep)

            # Advance time
            time = time + self.timestep

            # Sudden change
            if time > 0:
                self.autopilot.target_heading = 180
            if time > 5:
                self.autopilot.target_heading = 270
            if time > 8:
                self.autopilot.target_heading = 0
            if time > 40:
                self.autopilot.target_heading = 35
            if time > 60:
                self.autopilot.target_heading = 76
            if time > 90:
                self.autopilot.target_heading = None
                self.autopilot.target_power = 0

            # Unpack positions
            position_x, position_y = position

            # Append the data
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
                ('target_heading', self.autopilot.target_heading) ]
            for data_type, value in data_to_add:
                self.add_data(data_type, value)

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
        ax1.axhline(y=maximum_speed, color='C0', linestyle='-', linewidth=10, alpha=0.15, label='Max Target Speed')
        ax1.axhline(y=0, color='k', linestyle='-', linewidth=1, alpha=0.15)
        ax2.axvline(x=10, color='k', linestyle='-', linewidth=5, alpha=0.1)
        ax1.set_ylabel('Speed and Acceleration')
        ax1.set_ylim([-1,7])
        ax1.set_xlim([0,np.max(self.data['time'])])
        ax1.set_title(fr'$Kp$={self.autopilot.kp}, $Kd$={self.autopilot.kd}, $Ki$={self.autopilot.ki}')
        ax1.legend()

        # Plot headings on the second subplot

        # Replace the None when autopilot is off for Nans
        self.data['target_heading'] = [val if val is not None else np.nan for val in self.data['target_heading']]
        heading_plot_ylimits = [np.min([np.min(self.data['angle'])-10, np.nanmin(self.data['target_heading'])]) , \
                                np.max([np.max(self.data['angle'])+10, np.nanmax(self.data['target_heading'])])]

        ax2.plot(self.data['time'], self.data['angle'], label='Kayak', linewidth=5)
        ax2.axvline(x=10, color='k', linestyle='-', linewidth=5, alpha=0.1)
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
        ax2.yaxis.set_major_formatter(FuncFormatter(wrap_angles))

        ax2.legend(loc='upper right')


        # Plot engine powers on the third subplot
        ax3.plot(self.data['time'], self.data['right_engine_power'], color='r', label='Right')
        ax3.plot(self.data['time'], self.data['left_engine_power'],  color='b', label='Left')
        ax3.axvline(x=10, color='k', linestyle='-', linewidth=5, alpha=0.1)
        ax3.set_ylabel('Engine Power (%)')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylim([-5,105])
        ax3.set_xlim([0,np.max(self.data['time'])])
        ax3.legend()
        plt.tight_layout()

        plt.show()

        return





# Example simulation below

from kayak import Kayak
from weather import Weather
from autopilot import Autopilot

# Initialize weather
weather = Weather(wind_speed=12, wind_heading=170)

# Initialize kayak
kayak = Kayak(initial_speed=0, initial_heading=180, weather=weather)

# Initialize autopilot (kp 16 and kd 45 is a good start)
autopilot = Autopilot(kp=16, kd=45, ki=0, target_power=50, target_heading=180)

# Initialize simulator
simulator = Simulator(simulated_time=120, kayak=kayak, weather=weather, autopilot=autopilot, plot=True)

data = simulator.simulate_kayak()





# Animate position below

from matplotlib.animation import FuncAnimation

def update_plot(frame):
    # Choose the update interval (e.g., update every 5 frames)
    update_interval = 15
    sc = None
    # Only update the plot when the frame is a multiple of the update_interval
    if frame % update_interval == 0 or frame == len(data['time']) - 1:
        # Clear the previous scatter object if it exists
        if sc is not None:
            sc.remove()
        # Get the colormap and norm
        cmap = plt.get_cmap('viridis')
        norm = plt.Normalize(np.min(data['speed']), np.max(data['speed']))
        # Get the color for each point based on speed
        color = cmap(norm(data['speed'][frame]))
        # Use scatter to plot the data with color based on speed
        sc = ax.plot(data['position_x'][frame], data['position_y'][frame], linestyle='', marker='o', color=color)
        # sc = ax.scatter(data['position_x'][frame + 1], data['position_y'][frame + 1], color=color, marker='o')
        # Timer text
        timer_text.set_text('Time: {:.2f} seconds'.format(data['time'][frame]))

    return sc  # Return the scatter object to be used by the colorbar



# Create a figure
fig, ax = plt.subplots(figsize=(10, 8))  # Adjust width and height as needed


# Set axis limits (adjust according to your data range)
ax.set_xlim([np.min(data['position_x'])-2, np.max(data['position_x'])+2])
ax.set_ylim([np.min(data['position_y'])-2, np.max(data['position_y'])+2])
ax.set_ylabel('South-North (m)')
ax.set_xlabel('West-East (m)')
ax.set_aspect('equal')
ax.grid(True)

# Add the timer text
timer_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12, color='black')

# Create the animation
animation = FuncAnimation(fig, update_plot, frames=len(data['time']), interval=5, repeat=False)

# Get the colormap and norm
cmap = plt.get_cmap('viridis')
norm = plt.Normalize(np.min(data['speed']), np.max(data['speed']))

# Create a ScalarMappable with the colormap and norm
sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)

# Set the data for the ScalarMappable, but we only need the colorbar, so an arbitrary data point is sufficient
sm.set_array(data['speed'])

# Add a colorbar next to the plot
cax = plt.axes([0.92, 0.1, 0.02, 0.8])  # [left, bottom, width, height]
cbar = plt.colorbar(sm, cax=cax)  # Use the ScalarMappable for the colorbar
cbar.set_label('Speed (km/h)')

# To save the animation as a video file (optional)
# animation.save('position_animation.mp4', writer='ffmpeg')

# Show the animation
plt.show()








# Fastest time for 90 deg turn is 8.80 s

# KP 10, KD 85
