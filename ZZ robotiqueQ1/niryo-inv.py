from sympy import symbols, simplify, nsolve, diff
from sympy import Eq, cos, sin
from sympy import pi
import numpy as np


# For this problem, we will only solve for the position, and not the orientation.
# For this, we fix the 3 last joints (wrist in straight position, can be edited).


# Define symbolic variables

theta1, theta2, theta3 = symbols('theta1 theta2 theta3')  # Joint angles
theta4, theta5, theta6 = 0,0,0  # Fixed joint angles for wrist in radians

def DH_transform(a, alpha, d, theta):
    return np.array([
        [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0,           sin(alpha),             cos(alpha),            d],
        [0,           0,                      0,                     1]
    ])


# Numerical values for Cartesian position
x = 183
y = 183
z = 53

# Define the transformation matrices using the DH convention
#     DH_transform(   a, alpha,     d,         theta)
T01 = DH_transform(   0,  pi/2,   183,        theta1)
T12 = DH_transform( 210,     0,     0, theta2 + pi/2)
T23 = DH_transform(  30,  pi/2,     0,        theta3)
T34 = DH_transform(   0, -pi/2, 221.5,        theta4)
T45 = DH_transform(-5.5,  pi/2,     0,        theta5)
T56 = DH_transform(   0,     0,  23.7,        theta6)

# final rotation of the end effector frame (not following the DH convention)
T56 = T56 @ np.array([
    [0, 0, 1, 0],
    [0,-1, 0, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
]) 
    
#Compute the transformation matrix from frame 0 to frame 6
T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56
print(T06)

# Extract the position from the transformation matrix
position = T06[:3, 3]
print(simplify(position)) #"simplify" is used to simplify the expression


# Now we have a set of 3 equations linking the position (x,y,z) to the joint angles (theta1, theta2, theta3)


# Provide an initial guess for theta1 theta2 and theta3 (in radians)
# The goal here is to avoid singularities. The solver should keep the provided configuration(elbow down/up etc.).
initial_guess = (0.0, 0.0, 0.1)

# numericallly solve the equations
solutions = nsolve([Eq(x, position[0]), Eq(y, position[1]), Eq(z, position[2])], (theta1, theta2, theta3), initial_guess)

# Final pose
final_pose = [float(solutions[0]), float(solutions[1]), float(solutions[2]), float(theta4), float(theta5), float(theta6)]
final_pose_deg = [float(np.rad2deg(angle)) for angle in final_pose]
print("Final pose (degrees):", [round(angle, 2) for angle in final_pose_deg])
