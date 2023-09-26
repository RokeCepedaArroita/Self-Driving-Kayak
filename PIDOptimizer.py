# Create virtual situation

from kayak import Kayak
from weather import Weather
from autopilot import Autopilot
from simulator import Simulator


# Timestep
timestep = 0.122 # s

# Initialize weather: wind limit 21 km/h
weather = Weather(wind_speed=0, wind_heading=180+90)

# Initialize kayak
kayak = Kayak(initial_speed=4, initial_heading=45, weather=weather)

# Define noise parameters for the Kalman filter and sensor data generation
sensor_noise_water = {'compass_heading':  1.1,  # deg, sensor sensitivity 1 standard deviation
                      'angular_speed'  : 0.45,  # deg/s, sensor sensitivity 1 standard deviation
                      'covariance'     :    0}  # cov(compass, angular_speed) in deg^2/s

# sensor_noise_small = {'compass_heading':  2.5,  # deg, sensor sensitivity 1 standard deviation
#                       'angular_speed'  : 0.84,  # deg/s, sensor sensitivity 1 standard deviation
#                       'covariance'     : -0.3}  # cov(compass, angular_speed) in deg^2/s
#
# sensor_noise_large = {'compass_heading':  5.0,  # deg, sensor sensitivity 1 standard deviation
#                       'angular_speed'  :  4.2,  # deg/s, sensor sensitivity 1 standard deviation
#                       'covariance'     :  -8.7}  # cov(compass, angular_speed) in deg^2/s

sensor_noise = sensor_noise_water


from ziegler_nichols import ziegler_nichols
ziegler_nichols(ku=10, Tu=13.32)



# Calm conditions below
autopilot = Autopilot(kp=2.00, kd=7.1, ki=0.38, timestep=timestep, deadband=None,
                      smoothing_time=0.1, derivative_smoothing_time=0.5, sensor_noise=sensor_noise,
                      target_power=50, target_heading=135, alpha_fading_memory=1.1, integral_activation_angle=20)

# Initialize simulator
simulator = Simulator(simulated_time=100, timestep=timestep, phase_delay=True,
                      kayak=kayak, weather=weather, autopilot=autopilot,
                      sensor_noise=sensor_noise, plot=True)

data = simulator.simulate_kayak()






















#######################################################################################



# Create wind disturbance
disturbance = {'time':         50,  # s, time at which disturbance is applied
               'wind_speed':   17,  # km/h
               'wind_heading': 90} # deg
disturbance = None



def analyse_simulation_data():
    ''' Calculate the transition time, energy usage and disturbance dealing time '''

    return

def steady_state_detector(simulator, maximum_deviation):
    ''' Returns the transition time at which the system reaches a steady state,
    defined by the time at which the system is within 1 degree of the target,
    after which deviations of only maximum_deviation (deg) occur. This is compared
    to the best possible time, computed using a simulation where the PID loop is
    as aggressive as possible '''


    # Settling time as within X%?? Set maximum overshoot of 10 deg??


    return transition_time


def energy_usage_transient():

    # Measure the energy usage above the target power in the transient phase


    return



def store_simulation_results():

    return
