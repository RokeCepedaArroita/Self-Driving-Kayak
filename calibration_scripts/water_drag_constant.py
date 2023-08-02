import numpy as np
import matplotlib.pyplot as plt


import math

a = 3.65/2
b = 1.08/2
mass = 150
water_density = 1010
effective_area = math.pi*a*b
volume_needed_to_displace = mass/water_density
height_under_water = volume_needed_to_displace/effective_area

print(f'effective_area = {effective_area} m^2, height sunken = {height_under_water*100} cm')


# If 4.8 cm under water, and typical width is 0.95 m, then the drag coefficient is
CdA = 0.101*0.206
A = height_under_water*0.95
drag_coeff = CdA/A

print(f'Drag coefficient is {drag_coeff}')

# Drag coefficient is about 0.46


water_density = 997 # kg/m^3, tests were conducted in fresh water


# Field test data
powers = [23.3 , 41.5, 73.2 , 98.7, 128.4, 146.3, 172.7, 174  ]
speeds = [2.95 , 3.08, 4.15 , 4.57, 5.23 , 5.74 , 5.03 , 5.14 ]
direcs = ['out', 'in', 'out', 'in', 'in' , 'out', 'in' , 'out']


# Correct the velocities
correction = 0.131 # km/h this value minimises the ACd standard deviation


# Calculate drag constant and efficiency
ACds = []
ACds_corrected = []
efficiencies = []
efficiencies_corrected = []
corrected_speeds = []
forces = []

for power, speed, direc in zip(powers, speeds, direcs):

    if direc == 'in':
        corrected_speed = speed + correction
    elif direc == 'out':
        corrected_speed = speed - correction

    corrected_speeds.append(corrected_speed)
    # Drag constant
    ACd           = 2 * power / (water_density * (speed/3.6)**3) # m^2
    ACd_corrected = 2 * power / (water_density * (corrected_speed/3.6)**3) # m^2
    ACds_corrected.append(ACd_corrected)
    ACds.append(ACd)
    # Efficiency
    efficiency = (1/speed) * power # Wh/km
    efficiency_corrected = (1/corrected_speed) * power # Wh/km
    efficiencies_corrected.append(efficiency_corrected)
    efficiencies.append(efficiency)

    force = power/(corrected_speed/3.6)
    forces.append(force)




# Make ACd plot
plt.figure(1)
plt.title('Drag constant')
plt.plot(powers,ACds,'o', color='r', label='Uncorrected')
plt.plot(powers,ACds_corrected,'o', color='g', label='Corrected')
plt.xlabel('Power (W)')
plt.ylabel('A Cd (m^2)')
plt.ylim([0,0.2])
plt.xlim([0, 200])
plt.axhline(y=np.mean(ACds), color='k', linestyle='-', linewidth=5, alpha=0.3)
plt.legend()

print(f'ACd constant is {np.mean(ACds_corrected)} +- {np.std(ACds_corrected)} ({np.std(ACds_corrected)/np.mean(ACds_corrected)*100}\% error)')



# Make force plot
plt.figure(5)
plt.title('Force and power')
plt.plot(powers,np.divide(forces, 9.81),'o', color='g', label='Corrected')
plt.xlabel('Power (W)')
plt.ylabel('Force (kg)')
plt.xlim([0, 200])
plt.ylim([0, 12])
plt.legend()

print(f'ACd constant is {np.mean(ACds_corrected)} +- {np.std(ACds_corrected)} ({np.std(ACds_corrected)/np.mean(ACds_corrected)*100}\% error)')







# Make efficiency plot!
plt.figure(2)
plt.title('Efficiency')
plt.plot(powers, efficiencies,'o', color='r', label='Uncorrected')
plt.plot(powers, efficiencies_corrected,'o', color='g', label='Corrected')
plt.xlabel('Power (W)')
plt.ylabel('Efficiency (Wh/km)')
plt.grid()
plt.ylim([0, np.max(efficiencies)*1.2])
plt.xlim([0, 200])
plt.legend()


plt.figure(3)
plt.title('Range')
plt.plot(powers, np.divide(640,efficiencies),'o', color='r', label='Uncorrected')
plt.plot(powers, np.divide(640,efficiencies_corrected),'o', color='g', label='Corrected')
plt.xlabel('Power (W)')
plt.ylabel('Range (km)')
plt.grid()
plt.ylim([0, np.max(np.divide(640,efficiencies))*1.2])
plt.xlim([0, 200])
plt.legend()








# Maximum speed measurement


def water_drag(kayak_speed):
    ''' Simple model for the drag force of the water. Make it independent of angle for now '''

    CdA = 0.101 # m^2 -- Experimentally measured from power and speed data. Error on CdA is 0.018 (18%)
    water_density = 1010.5  # kg/m^3, 997 for fresh water, 1025 for sea water
    drag_force = 0.5 * water_density * (kayak_speed/3.6)**2 * CdA # Newtons, speed needs to be in m/s
    power_required = drag_force*(kayak_speed/3.6)

    return drag_force, power_required


drag_forces = []
powers_required = []



for speed in np.linspace(0,10,21):
    drag_force, power_required = water_drag(speed)
    print(f'At {speed} km/h, force is {drag_force/9.81*0.206} kg, need {power_required} W. Efficiency accounted.')
    drag_forces.append(drag_force/9.81)
    powers_required.append(power_required)



## TODO: MEASURE MOTOR EFFICIENCY!!!!! MOTOR IS ONLY X% EFFICIENT, SO I NEED TO MATCH THE QX MOTOR CURVE TO THE 100% EFFICIENT FORCE
## THAT WILL GIVE ME AN EFFICIENCY PERCENTAGE. THEN MULTIPLY THIS EFFICIENCY TO THE WATER DRAG FORCE TO SCALE IT DOWN ACCORDINGLY!!

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Given data
theory_power = [0, 13.5*12, 16*14, 20.4*16]
theory_push  = [0, 2.19   , 2.58 , 3.20   ]

# Convert the data to numpy arrays
power = np.array(theory_power)
push = np.array(theory_push)

# Define the power law function
def power_law(x, a, b):
    return a * x**b

def power_law_fixed_index(x, a):
    return a * x**0.6666666

# Perform the curve fitting
params, covariance = curve_fit(power_law, power, push)
paramst, covariancet = curve_fit(power_law, powers_required, drag_forces)

paramsfix, covariancefix = curve_fit(power_law_fixed_index, power, push)


# Extract the parameters
a_fit, b_fit = params
a_fitt, b_fitt = paramst
a_fitfix = paramsfix

# Generate data for the best fit curve
x_fit = np.linspace(min(power), max(power), 100)
y_fit = power_law(x_fit, a_fit, b_fit)

x_fitt = np.linspace(min(powers_required), max(powers_required), 100)
y_fitt = power_law(x_fitt, a_fitt, b_fitt)

x_fitfix = np.linspace(min(power), max(power), 100)
y_fitfix = power_law(x_fitfix, a_fitfix, 0.6666666)

# Plot the data and the best fit curve
plt.figure(7)
plt.plot(power, push, 'o', label='Data')
plt.plot(powers_required, drag_forces, 'o', label='100% Efficiency Push')
plt.plot(x_fit, y_fit, label='Best Fit: {:.3f} * x^({:.3f})'.format(a_fit, b_fit))
plt.plot(x_fitt, y_fitt, label='Best Fit: {:.3f} * x^({:.3f})'.format(a_fitt, b_fitt))
plt.plot(x_fitfix, y_fitfix)#, color = 'r', label=f'Best Fit: {a_fitfix:.3f} * x^(0.667)')
plt.ylabel('Push (kg)')
plt.xlabel('Power (W)')
plt.legend()


plt.figure(8)
efficiency_space = np.linspace(0,200,201)

# Generate data for the best fit curve
theory_push    = power_law(efficiency_space, a_fitt, b_fitt)
realworld_push = power_law(efficiency_space, a_fit, b_fit)
efficiency     = np.divide(realworld_push,theory_push) * 100

plt.plot(efficiency_space, efficiency)
plt.plot(efficiency_space[40:170], efficiency[40:170], 'r', label = f'mean efficiency {np.mean(efficiency[40:170]):.1f}%')
plt.title('Efficiency in converting power to linear motion')
plt.ylabel('Motor Efficiency (%)')
plt.xlim([0,200])
plt.ylim([0,35])
plt.grid()
plt.legend()
plt.xlabel('Power (W)')
plt.show()

# The theoretical push power curve has an index of 0.546, and the data points definitely do not fit a 0.667 index!!
# This means that the motor has nonlinear efficiency
