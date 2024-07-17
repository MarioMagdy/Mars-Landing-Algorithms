import numpy as np
from scipy.linalg import solve_continuous_are
import csv
from scipy.optimize import fsolve

def read_guidance_output(file_path):
    data = np.genfromtxt(file_path, delimiter=',', skip_header=1)
    return data

def control_system(data):
    # Extract the last state vector
    last_state = data[-1, 1:]  # Skip the time column

    # Extract necessary parameters from the state vector
    position = last_state[:3]
    velocity = last_state[3:6]
    omega = last_state[6:9]
    aoa = last_state[9]
    sideslip = last_state[10]
    bank_angle = last_state[11]
    
    
    # Here, integrate your control logic using the extracted parameters
    # Example:
    print(f"Position: {position}")
    print(f"Velocity: {velocity}")
    print(f"Angular Rates: {omega}")
    print(f"Angle of Attack: {aoa}")
    print(f"Sideslip Angle: {sideslip}")
    print(f"Bank Angle: {bank_angle}")

    # Compute control inputs and actuate thrusters (placeholder example)
    control_inputs = np.zeros(4)  # Replace with actual control logic
    return control_inputs


# Define thruster positions and directions
P_new = np.array([
    [-658, 983, -780.85],
    [-658, -983, -780.85],
    [-658, 983, 974.15],
    [-658, -983, 974.15]
])

V = np.array([
    [4, -28, 30],
    [4, 28, 30],
    [4, -28, -30],
    [4, 28, -30]
])

# Function to read guidance data from CSV file
def read_guidance_data_from_csv(file_path):
    with open(file_path, mode='r') as file:
        reader = csv.DictReader(file)
        data = []
        for row in reader:
            data.append(row)
        return data
    
# Reading data from CSV
file_path = 'guidance_output.csv'
guidance_data = read_guidance_data_from_csv(file_path)

# Process each state vector and compute control commands
for data in guidance_data:
    state_vector = [float(data[key]) for key in data.keys()]
    x = np.array(state_vector)  # State vector
    # x= [r, lon, lat, v, flight path angle, heading angle, omega_x, omega_y, omega_z, alpha, sideslipe, bnk ]
    

sigma = x[11]  # Bank angle in radians
coeff_deriv_longitudinal = [0.1, 0.01]  # Example coefficients for dCL/dAoA and dCm/dAoA
coeff_deriv_lateral = [0.1, 0.01, 0.01, 0.1, 0.01, 0.1, 0.01, 0.01]  # Example coefficients for dCl/dAoS, dCl/da, etc.
rho = 0.02  # Example air density at reentry altitude
S = 15  # Example reference area in square meters
c = 3  # Example mean aerodynamic chord in meters
b = 5  # Example wingspan in meters
Iyy = 2500  # Example moment of inertia about the y-axis in kg*m^2
Ixx = 2500  # Example moment of inertia about the x-axis in kg*m^2
Izz = 2500  # Example moment of inertia about the z-axis in kg*m^2
m = 1000  # Example mass of the lander in kg
mu = 4.282837e13  # Mars gravitational parameter in m^3/s^2
AoA = 0.1  # Angle of attack in radians
CL = 0.5  # Lift coefficient

# Function to compute longitudinal control matrices
def longitudinal_control(x, sigma, coeff_deriv, rho, S, c, Iyy, m):
    A = np.zeros((2, 2))
    B = np.zeros((2, 2))  # Now B has two columns: elevator deflection and RCS torque
    qdyn = 0.5 * rho * x[3]**2  # dynamic pressure
    dCL_dAoA = coeff_deriv[0] * 180 / np.pi
    dCm_dAoA = coeff_deriv[1] * 180 / np.pi
    
    A[0, 1] = qdyn * S * c / Iyy * dCm_dAoA
    A[1, 0] = 1
    A[1, 1] = -qdyn * S / (m * x[3]) * dCL_dAoA * np.cos(sigma)**2
    
    B[0, 0] = 1 / Iyy  # Elevator deflection
    B[1, 1] = 1 / Iyy  # RCS torque about y-axis
    
    return A, B

# Function to compute lateral control matrices
def lateral_control(x, AoA, sigma, CL, coeff_deriv, rho, S, b, Ixx, Izz, m, mu):
    A = np.zeros((4, 4))
    B = np.zeros((4, 4))  # Now B has four columns: aileron, rudder, RCS x, and RCS z
    
    cg = np.cos(x[5])
    tg = np.tan(x[5])
    sa = np.sin(AoA)
    ca = np.cos(AoA)
    cs = np.cos(sigma)
    
    qdyn = 0.5 * rho * x[3]**2  # dynamic pressure
    g = mu / x[0]**2  # gravitational parameter
    L = qdyn * CL * S
    
    dCl_dAoS = coeff_deriv[3] * 180 / np.pi
    dCl_da = coeff_deriv[4]
    dCn_dAoS = coeff_deriv[5] * 180 / np.pi
    dCn_da = coeff_deriv[6]
    dCn_dr = coeff_deriv[7]
    
    A[0, 2] = qdyn * S * b / Ixx * dCl_dAoS
    A[1, 2] = qdyn * S * b / Izz * dCn_dAoS
    A[2, 0] = sa
    A[2, 1] = -ca
    A[2, 3] = (x[3] / x[0] - g / x[3]) * cg * cs
    A[3, 0] = -ca
    A[3, 1] = -sa
    A[3, 2] = -(L / (m * x[3]) * cs + (x[3] / x[0] - g / x[3]) * cg) * cs
    A[3, 3] = L / (m * x[3]) * tg * cs
    
    B[0, 0] = 1 / Ixx  # Aileron deflection
    B[1, 1] = 1 / Izz  # Rudder deflection
    B[2, 2] = 1 / Ixx  # RCS torque about x-axis
    B[3, 3] = 1 / Izz  # RCS torque about z-axis
    
    return A, B

# Function to compute LQR gain
def lqr(A, B, Q, R):
    # Solve the continuous time LQR controller
    # dx/dt = A x + B u
    # cost = integral (x.T*Q*x + u.T*R*u) dt
    X = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ X
    return K


# Compute longitudinal and lateral A and B matrices
#A_long, B_long = longitudinal_control(x, sigma, coeff_deriv_longitudinal, rho, S, c, Iyy, m)
#A_lat, B_lat = lateral_control(x, AoA, sigma, CL, coeff_deriv_lateral, rho, S, b, Ixx, Izz, m, mu)

# Define Q and R matrices for LQR based on your provided matrices

# Longitudinal control cost matrices
max_pitch_rate_deviation = 1.0
max_AoA_deviation = np.deg2rad(0.5)
max_elevator_deflection = 1.0
max_RCS_torque_y = 10400.0


Q_long = np.diag([1/max_pitch_rate_deviation**2, 1/max_AoA_deviation**2])
R_long = np.diag([1/max_elevator_deflection**2, 1/max_RCS_torque_y**2])

# Lateral control cost matrices
max_roll_rate_deviation = 1.0
max_yaw_rate_deviation = 1.0
max_sideslip_angle_deviation = 0.1
max_bank_angle_deviation = np.deg2rad(8)
max_aileron_deflection = 1.0
max_rudder_deflection = 1.0
max_RCS_torque_x = 1600.0
max_RCS_torque_z = 7600.0

Q_lat = np.diag([
    1/max_roll_rate_deviation**2, 1/max_yaw_rate_deviation**2,
    1/max_sideslip_angle_deviation**2, 1/max_bank_angle_deviation**2
])
R_lat = np.diag([
    1/max_aileron_deflection**2, 1/max_rudder_deflection**2,
    1/max_RCS_torque_x**2, 1/max_RCS_torque_z**2
])
"""
# Compute the LQR gains
K_long = lqr(A_long, B_long, Q_long, R_long)
K_lat = lqr(A_lat, B_lat, Q_lat, R_lat)

# Compute control inputs
u_long = -K_long @ x[:2]  # Longitudinal control input
u_lat = -K_lat @ x[:4]    # Lateral control input

# Extract forces and torques
force_long = u_long[0]
torque_long = u_long[1]
force_lat = u_lat[:2]
torque_lat = u_lat[2:]

force = np.array([force_long, force_lat[0], force_lat[1]])
torque = np.array([torque_lat[0], torque_long, torque_lat[1]])"""

# Directly compute thruster commands
def compute_thruster_commands(force, torque, P_new, V):
    FT = np.concatenate((force, torque))
    
    # Calculate the cross product for each pair
    cross_products = np.array([np.cross(P_new[i], V[i]) for i in range(len(P_new))])
    
    D = np.vstack((V.T, cross_products.T))
    
    thruster_cmds = np.linalg.lstsq(D, FT, rcond=None)[0]
    
    return thruster_cmds

#thrusters_cmd = compute_thruster_commands(force, torque, P_new, V)

# Reading data from CSV
file_path = 'guidance_output.csv'
guidance_data = read_guidance_data_from_csv(file_path)

# Process each state vector and compute control commands
for data in guidance_data:
    state_vector = [float(data[key]) for key in data.keys()]
    x = np.array(state_vector)  # State vector

    # Compute longitudinal and lateral A and B matrices
    A_long, B_long = longitudinal_control(x, sigma, coeff_deriv_longitudinal, rho, S, c, Iyy, m)
    A_lat, B_lat = lateral_control(x, AoA, sigma, CL, coeff_deriv_lateral, rho, S, b, Ixx, Izz, m, mu)

    # Compute the LQR gains
    K_long = lqr(A_long, B_long, Q_long, R_long)
    K_lat = lqr(A_lat, B_lat, Q_lat, R_lat)

    # Compute control inputs
    u_long = -K_long @ x[:2]  # Longitudinal control input
    u_lat = -K_lat @ x[:4]    # Lateral control input

    # Extract forces and torques
    force_long = u_long[0]
    torque_long = u_long[1]
    force_lat = u_lat[:2]
    torque_lat = u_lat[2:]

    force = np.array([force_long, force_lat[0], force_lat[1]])
    torque = np.array([torque_lat[0], torque_long, torque_lat[1]])

    # Compute thruster commands
    thrusters_cmd = compute_thruster_commands(force, torque, P_new, V)


print("Longitudinal A matrix:\n", A_long)
print("Longitudinal B matrix:\n", B_long)
print("Longitudinal K matrix:\n", K_long)
print("##########################################################################")
print("\n")

print("Lateral A matrix:\n", A_lat)
print("Lateral B matrix:\n", B_lat)
print("Lateral K matrix:\n", K_lat)
print("########################################################################")
print("\n")

# Print the control inputs and controlled bank angle
print("Longitudinal Control Inputs (Elevator Deflection, RCS Torque y-axis):", u_long)
print("Lateral Control Inputs (Aileron Deflection, Rudder Deflection, RCS Torque x-axis, RCS Torque z-axis):", u_lat)
print("############################################################################")
print("\n")


print("Thrusters Command:", thrusters_cmd)
print("Controlled Bank Angle:", sigma + u_lat[3])



