from math import sin,cos,pi
import numpy as np
# Define symbolic variables

theta1, theta2, theta3, theta4, theta5, theta6 = np.array([15,-30,-30,0,-30,0])*pi/180 # Joint angles

def DH_transform(a, alpha, d, theta):
    return np.array([
        [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0,           sin(alpha),             cos(alpha),            d],
        [0,           0,                      0,                     1]
    ])


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
print(position) 

# zero pose at point (245.2,0,417.5)
# [15 -30 -30 0 -30 0] at (228,61,164)