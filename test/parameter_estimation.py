import pandas as pd
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares
import matplotlib.pyplot as plt

measurement = pd.read_csv("./measurement_data4.csv")
sm_diff = measurement['sm'].diff()
time_diff = measurement['time'].diff()
measurement['speed'] = sm_diff / time_diff / 48
measurement['speed'].fillna(0, inplace=True)
print(measurement.head())

speed_name = 'speed'
V = 1.0

def system(t, y, b, J, K, L, R):
    theta_dot, i = y
    dtheta_dot_dt = -b/J * theta_dot + K/J * i
    di_dt = -K/L * theta_dot - R/L * i + 1/L * V
    return [dtheta_dot_dt, di_dt]

def simulate(params):
    b, J, K, L, R = params
    y0 = [0, 0]  # Initial values for theta' and i
    # t_span = (0, 10)  # From t=0 to t=10
    t_eval = measurement['time'].values  # Use the time column from measurement DataFrame
    t_span = (t_eval[0], t_eval[-1])  # From the first to the last time point
# np.linspace(t_span[0], t_span[1], 66)
    sol = solve_ivp(system, t_span, y0, t_eval=t_eval, args=(b, J, K, L, R))
    return sol

def objective(params, plot=False):
    sol = simulate(params)
    sim = pd.DataFrame({
        'time': sol.t,
        'theta_dot': sol.y[0],
        'i': sol.y[1]
    })
    area = 0.0
    for (i, theta_dot) in enumerate(sim['theta_dot'][1:]):
        if measurement['time'][i] < 0.2:
            area += abs(theta_dot - measurement[speed_name][i])
        else:
            area += abs(theta_dot - measurement[speed_name][i])
    if plot:
        plt.figure()
        plt.plot(sim['time'], sim['theta_dot'], label='Simulated theta_dot')
        plt.plot(measurement['time'], measurement[speed_name], label='Measured v')
        # plt.plot(measurement['time'], measurement['speed'], label='Calc v')
        plt.xlabel('Time')
        plt.ylabel('Value')
        plt.legend()
        plt.show()
        # sim.plot(y='theta_dot')
        # measurement.plot(y=speed_name)
        # plt.show()
    print(area)
    if area == 0:
        area = 100
    return area

# b, J, K, L, R
params = [0.08682405, 0.00258706, 0.11394443, 0.04805594, 0.07610658]

result = least_squares(objective, params, max_nfev=10000, ftol=1e-6, xtol=1e-6, gtol=1e-6)
params = result.x
print(f'Optimized parameters: {params}')

# Simulate with the optimized parameters
objective(params, plot=True)