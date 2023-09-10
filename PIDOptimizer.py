# Create virtual situation

from kayak import Kayak
from weather import Weather
from autopilot import Autopilot
from simulator import Simulator

# Initialize weather: wind limit 28 km/h
weather = Weather(wind_speed=17, wind_heading=180)

# Initialize kayak
kayak = Kayak(initial_speed=4, initial_heading=45, weather=weather)

# Initialize autopilot (Pessen: kp=49, kd=68, ki=13, Classic: kp=42, kd=49, ki=9, No overshoot: kp=14, kd=43, ki=3, Some Overshoot: kp=23, kd=72, ki=5)
autopilot = Autopilot(kp=49, kd=68, ki=13, smoothing_time=0.1, derivative_smoothing_time=0.4, target_power=50, target_heading=135)

# Create wind disturbance
disturbance = {'time':         50,  # s, time at which disturbance is applied
               'wind_speed':   17,  # km/h
               'wind_heading': 90} # deg

# Initialize simulator
simulator = Simulator(simulated_time=100, kayak=kayak, weather=weather, autopilot=autopilot, disturbance=disturbance, plot=True)

data = simulator.simulate_kayak()





def analyse_simulation_data():
    ''' Calculate the transition time, energy usage and disturbance dealing time '''

    return

def steady_state_detector(simulator, maximum_deviation):
    ''' Returns the transition time at which the system reaches a steady state,
    defined by the time at which the system is within 1 degree of the target,
    after which deviations of only maximum_deviation (deg) occur. This is compared
    to the best possible time, computed using a simulation where the PID loop is
    as aggressive as possible '''


    return transition_time


def energy_usage_transient():

    # Measure the energy usage above the target power in the transient phase


    return



def store_simulation_results():

    return
