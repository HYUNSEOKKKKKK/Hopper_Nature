import numpy as np
import matplotlib.pyplot as plt

def relaxed_log_barrier(delta, alpha_lower, alpha_upper, x):
    # Calculate the relaxed log barrier function
    x_temp = x - alpha_lower
    if x_temp < delta:
        y = 0.5 * (((x_temp - 2 * delta) / delta) ** 2 - 1) - np.log(delta)
    else:
        y = -np.log(x_temp)

    x_temp = -(x - alpha_upper)
    if x_temp < delta:
        y += 0.5 * (((x_temp - 2 * delta) / delta) ** 2 - 1) - np.log(delta)
    else:
        y += -np.log(x_temp)

    return -y  # Multiply by -1 as per the last line in your code

def relaxed_barrier(alpha_lower, alpha_upper, x):
    # Calculate the relaxed log barrier function
    # Initialize output
    y = 0.0

    delta = 1/10 * (alpha_upper-alpha_lower)
    # Calculate the relaxed barrier function
    # For the lower bound
    if x < alpha_lower + delta:
        x_temp = x - (alpha_lower + delta)
        y += (x_temp**2) / (2 * delta**2)  # Quadratic transition to zero

    # For the upper bound
    if x > alpha_upper - delta:
        x_temp = x - (alpha_upper - delta)
        y += (x_temp**2) / (2 * delta**2)  # Quadratic transition to zero

    # The region where y should be zero
    if alpha_lower + delta <= x <= alpha_upper - delta:
        y = 0.0

    return -y  # Multiply by -1 as per the last line in your code

# Set parameters
delta = 0.015
alpha_lower = -0.08
alpha_upper = 1.0

# Generate a range of x values
x_values = np.linspace(-1.0, 1.0, 400)  # Avoiding x=alpha_lower directly as it goes to -inf in log
# y_values = [relaxed_log_barrier(delta, alpha_lower, alpha_upper, x) for x in x_values]
# y_values = [relaxed_barrier(alpha_lower, alpha_upper, x) for x in x_values]
y_values = [np.exp(-1*x**2) for x in x_values]

# Plot the function
plt.plot(x_values, y_values)
# plt.xlabel('x')
# plt.ylabel('y')
# plt.title('Relaxed Log Barrier Function')
# plt.legend()
# plt.grid(True)
plt.show()