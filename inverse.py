from sympy import symbols, Eq, solve, cos, sin, atan2, sqrt
from math import pi
# Define symbolic variables
theta1, theta2 = symbols('theta1 theta2')  # Joint angles

# Numerical values for Cartesian position
x = 30
y = 35

# Numerical values for link lengths
a1 = 20
a2 = 30

# Forward kinematics equations
x_eq = a1 * cos(theta1) + a2 * cos(theta1 + theta2)
y_eq = a1 * sin(theta1) + a2 * sin(theta1 + theta2)

# Solve for theta2
cos_theta2 = (x**2 + y**2 - a1**2 - a2**2) / (2 * a1 * a2)
theta2_sol = atan2(sqrt(1 - cos_theta2**2), cos_theta2)  # Elbow-up solution

# Solve for theta1
k1 = a1 + a2 * cos(theta2_sol)
k2 = a2 * sin(theta2_sol)
theta1_sol = atan2(y, x) - atan2(k2, k1)

# Display the solutions
print("Theta1 solution:", theta1_sol)
print("Theta2 solution:", theta2_sol)

# Solve the system of equations [x_eq, y_eq] for theta1 and theta2
from sympy import nsolve

# Provide an initial guess for theta1 and theta2
initial_guess = (0.5, 0.5)
solutions = nsolve([Eq(x, x_eq), Eq(y, y_eq)], (theta1, theta2), initial_guess)

# Display the solutions
print("Solutions for theta1 and theta2:")
print(f"Theta1: {solutions[0]*180/pi}, Theta2: {solutions[1]*180/pi}")