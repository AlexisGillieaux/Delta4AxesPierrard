from sympy import symbols, simplify, nsolve, diff
from sympy import Eq, cos, sin
from sympy import pi
import numpy as np
# Define symbolic variables

theta1, theta2, theta3 = symbols('theta1 theta2 theta3')  # Joint angles

def DH_transform(a, alpha, d, theta):
    return np.array([
        [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0,           sin(alpha),             cos(alpha),            d],
        [0,           0,                      0,                     1]
    ])


# Numerical values for Cartesian position
x = 2
y = 2
z = 3

# Numerical values for link lengths
l1 = 4
l2 = 3
l3 = 2

# Define the transformation matrices using the DH convention
#     DH_transform(  a,   alpha,   d,         theta)
T01 = DH_transform(  0,   -pi/2,  l1,   theta1+pi/2)
T12 = DH_transform( l2,       0,   0,        theta2)
T23 = DH_transform( l3,       0,   0,        theta3)

# final rotation of the end effector frame (not following the DH convention)
T23 = T23 @ np.array([
    [0, 0, 1, 0],
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]
]) 
    
#Compute the transformation matrix from frame 0 to frame 3
T03 = T01 @ T12 @ T23
print(T03)

# Extract the position from the transformation matrix
position = T03[:3, 3]
print(simplify(position)) #"simplify" is used to simplify the expression



# Provide an initial guess for theta1 theta2 and theta3 (in radians)
# The goal here is to avoid singularities. The solver should keep the provided configuration(elbow down/up etc.).
initial_guess = (0.0, 0.0, 0.1)
solutions = nsolve([Eq(x, position[0]), Eq(y, position[1]), Eq(z, position[2])], (theta1, theta2, theta3), initial_guess)

# Display the solutions
print("Solutions for theta1, theta2, and theta3:")
print(f"Theta1: {np.rad2deg(float(solutions[0]))}")
print(f"Theta2: {np.rad2deg(float(solutions[1]))}")
print(f"Theta3: {np.rad2deg(float(solutions[2]))}")
