import pandas as pd
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares
import matplotlib.pyplot as plt

measurement = pd.read_csv("./measurement_data.csv")
V = 3.3

def system(t, y, b, J, K, L, R):
    theta_dot, i = y
    dtheta_dot_dt = -b/J * theta_dot + K/J * i
    di_dt = -K/L * theta_dot - R/L * i + 1/L * V
    return [dtheta_dot_dt, di_dt]

def simulate(params):
    b, J, K, L, R = params
    y0 = [0, 0]  # Initial values for theta' and i
    t_span = (0, 10)  # From t=0 to t=10
    sol = solve_ivp(system, t_span, y0, t_eval=np.linspace(t_span[0], t_span[1], 66), args=(b, J, K, L, R))
    return sol

def objective(params, plot=False):
    sol = simulate(params)
    sim = pd.DataFrame({
        'time': sol.t,
        'theta_dot': sol.y[0],
        'i': sol.y[1]
    })
    area = 0.0
    for (i, theta_dot) in enumerate(sim['theta_dot']):
        if np.isfinite(measurement['v'][i]):
            area += theta_dot - measurement['v'][i]
    if plot:
        sim.plot(y='theta_dot')
        measurement.plot(y='v')
        plt.show()
    print(area)
    return area

initial_guess = [1.5319e-04, 1.2130e-05, 0.0034, 0.0127, 0.0710]
result = least_squares(objective, initial_guess)
optimized_params = result.x
print(f'Optimized parameters: {optimized_params}')

# Simulate with the optimized parameters
objective(optimized_params, plot=True)
# sim = pd.DataFrame({
#     'time': sol.t,
#     'theta_dot': sol.y[0],
#     'i': sol.y[1]
# })

# # Plot the results
# plt.figure()
# plt.plot(sim['time'], sim['theta_dot'], label='Simulated theta_dot')
# plt.plot(measurement['time'], measurement['theta_dot'], label='Measured theta_dot')
# plt.legend()
# plt.xlabel('Time')
# plt.ylabel('theta_dot')
# plt.title('Comparison of Simulated and Measured theta_dot')
# plt.show()