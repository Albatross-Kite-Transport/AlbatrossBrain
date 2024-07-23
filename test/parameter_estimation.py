import pandas as pd
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares, differential_evolution
import matplotlib.pyplot as plt
import traceback

measurements = []
measurements.append(pd.read_csv("./smooth_middle_3.csv"))
# measurements.append(pd.read_csv("./smooth_middle.csv"))
force_time_diff = measurements[-1]['time'].diff()

m = 25.4/9.81

# 25.4
"""
force = m * a
"""
vm_diff = measurements[-1]['vm'].diff()
measurements[-1]['force'] = (-vm_diff / force_time_diff + 9.81)* m
measurements[-1]['force'].fillna(0, inplace=True)

speed_name = 'vm'

V = -1.0
force = 0.0
diameter = 0.110
circumference = np.pi * diameter
trashbin = False
iteration = 0
    # params = [1.8757378888984615, 0.06544518834061552, 0.005018645219080575, 0.008694475824383652, 0.17940812330950706, 0.3485777609153695, 1.964075144909581] # good estimation of speed

"""
b, r, J, K, L, R
r: radius
"""

bounds = [(0, 1.0), (0, 1.0), (0, 1.0), (0, 1.0), (0, 1.0), (0, 1.0), (0, 10.0)]  # Adjust these bounds as necessary

if not trashbin:
    params = [0.03755102000263477, 0.6388008980489319, 9.045715356402717, 0.0011604215466274326, 0.028818868274443687, 1.8742265685593478]
    params = [1.37558490e-01, 4.44401761e-03, 3.24033025e-04, 4.80614776e-07, 2.28836786e-01, 4.62923268e-01, 2.02119001e+00] # latest
    params = [0.9677403396729308, 0.2612447328622712, 0.004285832333671802, 0.0006277748456337551, 0.004607088441398843, 0.008432584474247717, 9.89609227861451]
else:
    params = [0.12579665, 0.01675235, 0.00319081, 0.25413181, 0.0858939, 0.11507999] # very good params for trashbin. only a is important here


# params_to_optimize = [1e-6, 1e-6]
params_to_optimize = params

"""
dtheta_dot_dt: rotations/s
"""
def system(t, y, params, cache=[0,0]):
    theta_dot, i = y
    last_dtheta_dot_dt, last_theta_dot = cache
    """
    trashbin:
    dtheta_dot_dt = (-m * dtheta_dot_dt * a - b*theta_dot + K*i) / J
    dtheta_dot_dt * J = -m * dtheta_dot_dt * a - b*theta_dot + K*i
    dtheta_dot_dt * J + m * dtheta_dot_dt * a = -b*theta_dot + K*i
    dtheta_dot_dt = (-b*theta_dot + K*i) / (J + m*a)

    force:
    torque = force * a
    dtheta_dot_dt = (force*a - b*theta_dot + K*i) / J
    """
    measure = params[-1]
    force = 0.0
    # if 'force' in measure.columns:
    #     # nearest_time_index = np.abs(measure['time'] - t).idxmin()
    #     # force = measure['force'][nearest_time_index]
    #     if last_dtheta_dot_dt > 0.0 and last_theta_dot > 0.0:
    #         force = m * last_dtheta_dot_dt + 10*last_theta_dot
    if 'force' in measure.columns:
        """
        dtheta_dot_dt + m * dtheta_dot_dt / params[0] = - theta_dot / params[1] * params[6] + i / params[2]
        dtheta_dot_dt = (-theta_dot / params[1] * params[6] + i / params[2]) / (1 + m/params[0])
        """
        dtheta_dot_dt = (-theta_dot / params[1] * params[6] + i / params[2]) / (1 + m/params[0])
    else:
        dtheta_dot_dt = -force / params[0] - theta_dot / params[1] + i / params[2]
    di_dt = params[3]* theta_dot - i / params[4] + V / params[5]
    cache[0], cache[1] = dtheta_dot_dt, theta_dot
    return [dtheta_dot_dt, di_dt]

def simulate(params, smooth=False):
    measure = params[-1]
    y0 = [0, 0]  # Initial values for theta' and i
    # t_span = (0, 10)  # From t=0 to t=10
    t_eval = measure['time'].values  # Use the time column from measurements DataFrame
    t_span = (t_eval[0], t_eval[-1])  # From the first to the last time point
        
# `method` must be one of {'RK23': <class 'scipy.integrate._ivp.rk.RK23'>, 'RK45': <class 'scipy.integrate._ivp.rk.RK45'>, 'DOP853': <class 'scipy.integrate._ivp.rk.DOP853'>, 'Radau': <class 'scipy.integrate._ivp.radau.Radau'>, 'BDF': <class 'scipy.integrate._ivp.bdf.BDF'>, 'LSODA': <class 'scipy.integrate._ivp.lsoda.LSODA'>} or OdeSolver class.
    sol = solve_ivp(system, t_span, y0, t_eval=t_eval, args=(params,), method='BDF', rtol=1e-7)
    return sol

def objective(obj_params, plot=False, smooth=False):
    global params, iteration
    if len(obj_params) == 2:
        params[0] = obj_params[0]
        params[1] = obj_params[1]
    else:
        params = obj_params
    if not isinstance(params, list):
        params = params.tolist()
    params.append(measurements)
    solutions = []
    try:
        for measure in measurements:
            params[-1] = measure
            solutions.append(simulate(params))
    except (ValueError, ZeroDivisionError) as e:
        print(e)
        traceback.print_exc()
        return np.inf
    area = 0.0
    try:
        for (sol, measure) in zip(solutions, measurements):
            sim = pd.DataFrame({
                'time': sol.t,
                'theta_dot': sol.y[0],
                'i': sol.y[1],
            })
            if not smooth:
                for (theta_dot, speed, i, im, t) in zip(sim['theta_dot'], measure[speed_name], sim['i'], measure['Im'], measure['time']):
                    # if measurements[0]['time'][i] > 3.0:
                    #     area += abs(theta_dot - measurements[speed_name][i])*10
                    # else:
                    #     area += abs(theta_dot - measurements[speed_name][i])
                    if t < 1.0:
                        area += (abs(theta_dot - speed) + abs(i - im)*10)
                    else:
                        area += (abs(theta_dot - speed) + abs(i - im)*10)
            if plot:
                plt.figure()
                # plt.plot(sim['time'], sim['theta_dot'], label='Simulated theta_dot')
                # plt.plot(sim['time'], sim['i'], label='Simulated i')
                plt.plot(measure['time'], measure[speed_name], label='Measured vm')
                plt.plot(measure['time'], measure['Im'], label='Measured i')
                plt.plot(measure['time'], measure['am']/30, label='Measured am')
                plt.plot(measure['time'], measure['sm']/100, label='Measured sm')
                if 'force' in measure.columns:
                    plt.plot(measure['time'], measure['force']/100, label='Measured force')
                # plt.plot(measurements[0]['time'], measurements[speed_name], label='Calc v')
                plt.xlabel('Time')
                plt.ylabel('Value')
                plt.legend()
                plt.show()
                # sim.plot(y='theta_dot')
                # measurements.plot(y=speed_name)
                # plt.show()
    except OverflowError as _:
        area = np.inf

    if area == 0.0:
        area = np.inf
    iteration += 1
    if iteration % 100 == 0:
        print(area, "\t", params[:-1], "\t", iteration)
    return area

optimized_params = params
# b, r, J, K, L, R
# params_to_optimize = [p + np.random.uniform(-0.1, 0.1) for p in params_to_optimize]

# print(params_to_optimize)
# # result = least_squares(objective, params_to_optimize, max_nfev=10000, ftol=1e-8, xtol=1e-8, gtol=1e-8)
# result = differential_evolution(objective, bounds, maxiter=5000, tol=1e-5)
# optimized_params = result.x
# print(f'Optimized parameters: {optimized_params}')

# Simulate with the optimized parameters
objective(optimized_params, plot=True, smooth=False)