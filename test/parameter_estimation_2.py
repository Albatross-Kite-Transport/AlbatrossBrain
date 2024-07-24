import pandas as pd
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares, differential_evolution
import matplotlib.pyplot as plt
import traceback

measurements = []
measurements.append(pd.read_csv("./pid-force.csv"))
measurements.append(pd.read_csv("./constant-force.csv"))
measurements.append(pd.read_csv("./pid-filtered.csv"))
# measurements.append(pd.read_csv("./measurement_data_trash_filter_i_3.csv"))

masses = [0, (10.7 + 11.7) / 2, 22.5]

# 25.4
"""
force = m * a
"""

for (measure, mass) in zip(measurements, masses):
    measure.ffill(inplace=True)
    if 'force' not in measure.columns:
        measure['force'] = mass
    # measure['vm'] = measure['vm'].rolling(window=10, center=False).mean()
    # measure['am'] = measure['am'].rolling(window=10, center=False).mean()
    # measure['am'] = abs(measure['am'])
    print(measure.head())


# for (measure, mass) in zip(measurements, masses):
#     if mass > 0:
#         measure['a_trash'] = measure['am']
#     else:
#         measure['a_trash'] = 0.0

speed_name = 'vm'

# V = -1.0
# force = 0.0
# diameter = 0.110
# circumference = np.pi * diameter
iteration = 0

"""
b, r, J, K, L, R
r: radius
"""

bounds = [(-10.0, 10.0), (-10.0, 10.0), (-10.0, 10.0), (-10.0, 10.0)]  # Adjust these bounds as necessary

params = [0.12346058735337806, 0.003842775428916312, 0.00028357567370296266] # latest


# params_to_optimize = [1e-6, 1e-6]
params_to_optimize = params

"""
dtheta_dot_dt: rotations/s
"""
"""
dtheta_dot_dt = force / params[0] + theta_dot / params[1] - i / params[2]
force / params[0] = dtheta_dot_dt - theta_dot / params[1] + i / params[2]
force = dtheta_dot_dt*params[0] - theta_dot * params[1] + i * params[2]
"""

def objective(obj_params, plot=False, smooth=False):
    global params, iteration
    params = obj_params
    total_cost = 0.0
    try:
        for (measure, m) in zip(measurements, masses):
            f_estimateds = []
            if not smooth:
                for (f_measured, a_motor, v, V, i, t) in zip(measure['force'], measure['am'], measure['vm'], measure['m'], measure['Im'], measure['time']):
                    """
                    force = dtheta_dot_dt*params[0] - theta_dot * params[1] + i * params[2]


                    cost = abs(force_calculated - force_formula)
                    cost = m * a - (a*params[0] - v*params[1] + i*params[2])
                    """
                    if np.isfinite(a_motor) and np.isfinite(v):
                        f_estimated = a_motor*params[0] + v/params[1] + i/params[2]
                        # f_measured = -v/params[1] + i/params[2]
                        f_estimateds.append(f_estimated)
                        # print(f_measured)
                        # print(f_estimated)
                        # print(abs(float(f_measured) - f_estimated))
                        # [ 0.03617786 -3.05630261  0.07395525 -0.01084698]
                        if m==(10.7 + 11.7) / 2:
                            if 317 < t < 318.0:
                                total_cost += abs(float(f_measured) - f_estimated)*19**2
                        elif m==22.5:
                            if 78 < t < 80:
                                total_cost += abs(float(f_measured) - f_estimated)*10**2
                        else:
                            total_cost += abs(float(f_measured) - f_estimated)**2
                    else:
                        f_estimateds.append(0.0)

            if plot:
                plt.figure()
                # plt.plot(measure['time'], measure['force'], label='Measured f')
                # plt.plot(measure['time'], f_estimateds, label='Estimated f')
                # plt.plot(measure['time'], measure['Im'], label='i')
                # plt.plot(measure['time'], measure['vm'], label='vm')
                # plt.plot(measure['time'], measure['am'], label='am')
                plt.plot(measure['time'], measure['sm'], label='sm')
                # plt.plot(measure['time'], measure['a_trash'], label='a_trash')
                # if 'force' in measure.columns:
                #     plt.plot(measure['time'], measure['force']/100, label='Measured force')
                # plt.plot(measurements[0]['time'], measurements[speed_name], label='Calc v')
                plt.xlabel('Time')
                plt.ylabel('Value')
                plt.legend()
                plt.show()
                # sim.plot(y='theta_dot')
                # measurements.plot(y=speed_name)
                # plt.show()
    except OverflowError as _:
        total_cost = np.inf

    if total_cost == 0.0:
        total_cost = np.inf
    iteration += 1
    if iteration % 100 == 0:
        print(total_cost, "\t", params, "\t", iteration)
    return total_cost

optimized_params = params
# b, r, J, K, L, R
# params_to_optimize = [p + np.random.uniform(-0.1, 0.1) for p in params_to_optimize]

# print(params_to_optimize)
# # result = least_squares(objective, params_to_optimize, max_nfev=10000, ftol=1e-8, xtol=1e-8, gtol=1e-8)
# result = differential_evolution(objective, bounds, maxiter=10000, tol=1e-8)
# optimized_params = result.x
# print(f'Optimized parameters: {optimized_params}')

# Simulate with the optimized parameters
objective(optimized_params, plot=True, smooth=False)