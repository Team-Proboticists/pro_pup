import numpy as np
from scipy.optimize import fsolve

# System Variables
L1, L2, L3, d1, d2 = 95.5, 100, 144.351, 10.191, 40

# Forward Kinematics
def kinematic_matrix(L1, L2, L3, theta1, theta2, theta3, d1, d2):
    # Convert angles from degrees to radians
    theta1 = np.radians(theta1)
    theta2 = np.radians(theta2)
    '''
    Important Please Note: Theta 3 is modified here to suit the motor 3 coordinate system in actual implementation
    '''
    theta3 = theta3 - 180
    theta3 = np.radians(theta3)
    
    # Define the position of the end effector    
    Qx = L2*np.cos(theta1)*np.cos(theta2) + L3*np.cos(theta1)*np.cos(theta2+theta3) + d1*np.cos(theta1) + d2*np.sin(theta1)
    Qy = L2*np.sin(theta1)*np.cos(theta2) + L3*np.sin(theta1)*np.cos(theta2+theta3) + d1*np.sin(theta1) - d2*np.cos(theta1)
    Qz = L1 + L2*np.sin(theta2) + L3*np.sin(theta2+theta3)
    return Qx, Qy, Qz
# Inverse Kinematics
def inverse_kinematics(L1, L2, L3, d1, d2, Qx, Qy, Qz):
    def equations(vars):
        theta1, theta2, theta3 = vars
        eq1 = Qx - (L2 * np.cos(theta1) * np.cos(theta2) + 
                    L3 * np.cos(theta1) * np.cos(theta2 + theta3) +
                    d1 * np.cos(theta1) + d2 * np.sin(theta1))
        eq2 = Qy - (L2 * np.sin(theta1) * np.cos(theta2) + 
                    L3 * np.sin(theta1) * np.cos(theta2 + theta3) +
                    d1 * np.sin(theta1) - d2 * np.cos(theta1))
        eq3 = Qz - (L1 + L2 * np.sin(theta2) + L3 * np.sin(theta2 + theta3))
        return [eq1, eq2, eq3]

    # Initial guess to begin the solve
    initial_guess = [0, 0, 0]
    # Solve the equations using the initial guess
    solution = fsolve(equations, initial_guess)
    theta1, theta2, theta3 = solution

    # Normalize angles to desired ranges
    def normalize_angle(angle, lower, upper):
        angle = angle % 360
        while angle < lower:
            angle += 360
        while angle >= upper:
            angle -= 360
        return angle
    
    # Convert to degrees and normalize
    theta1 = normalize_angle(np.degrees(theta1), 0, 360)         # Restrict theta1 to [0, 90]
    theta2 = normalize_angle(np.degrees(theta2), 0, 180)  # [0, 180] for theta2
    theta3 = normalize_angle(np.degrees(theta3), -180, 180) # [-180, 180] for theta3
    '''
    Important Please Note: Theta 3 is modified here to suit the motor 3 coordinate system in actual implementation
    '''
    theta3 += 180  # Adjust theta3 to match the Motor 3 coordinate system
    # Round the angles to 2 decimal places
    theta1 = round(theta1, 2)
    theta2 = round(theta2, 2)
    theta3 = round(theta3, 2)
    # If any angle is 360 make it zero
    if theta1 == 360:
        theta1 = 0
    if theta2 == 360:
        theta2 = 0
    if theta3 == 360:
        theta3 = 0

    return theta1, theta2, theta3

# Test the functions
# theta1, theta2, theta3 = 0, 65.13, 44.19
# Qx, Qy, Qz = 100, -40, 50

# # Compute forward kinematics
# coordinates = kinematic_matrix(L1, L2, L3, theta1, theta2, theta3, d1, d2)
# coordinates = tuple(map(lambda x: round(x, 2), coordinates))
# print("Coordinates: ", coordinates)

# # Solve Inverse Kinematics
# angles= inverse_kinematics(L1, L2, L3, d1, d2, Qx, Qy, Qz)
# print("Inverse Kinematics Angles: ", angles)

# commands = [(50, -40, 300), (120, 0, 160), (200, 90, 160), (230, 0, 100)]
# angles_list = []
# for Qx, Qy, Qz in commands:
#     angles = inverse_kinematics(L1, L2, L3, d1, d2, Qx, Qy, Qz)
#     angles_list.append(angles)
#     print(f"Coordinates: ({Qx}, {Qy}, {Qz}) -> Inverse Kinematics Angles: {angles}")