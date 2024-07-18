import numpy as np
from scipy.linalg import solve_continuous_are

class ControlSystem:
    def __init__(self, P_new, V, x, sigma, coeff_deriv_longitudinal, coeff_deriv_lateral, 
                 rho, S, c, b, Iyy, Ixx, Izz, m, mu, AoA, CL):
        self.P_new = P_new
        self.V = V
        self.x = x
        self.sigma = sigma
        self.coeff_deriv_longitudinal = coeff_deriv_longitudinal
        self.coeff_deriv_lateral = coeff_deriv_lateral
        self.rho = rho
        self.S = S
        self.c = c
        self.b = b
        self.Iyy = Iyy
        self.Ixx = Ixx
        self.Izz = Izz
        self.m = m
        self.mu = mu
        self.AoA = AoA
        self.CL = CL
        self.thruster_limit = 1e3  # Adjust this value based on actual thruster capabilities

        # Longitudinal and lateral control cost matrices
        self.Q_long = np.diag([1/1.0**2, 1/np.deg2rad(0.5)**2])
        self.R_long = np.diag([1/1.0**2, 1/10400.0**2])
        self.Q_lat = np.diag([
            1/1.0**2, 1/1.0**2, 1/0.1**2, 1/np.deg2rad(8)**2
        ])
        self.R_lat = np.diag([
            1/1.0**2, 1/1.0**2, 1/1600.0**2, 1/7600.0**2
        ])

        self.K_long = self.compute_lqr(*self.longitudinal_control())
        self.K_lat = self.compute_lqr(*self.lateral_control())

    def longitudinal_control(self):
        A = np.zeros((2, 2))
        B = np.zeros((2, 2))  # Now B has two columns: elevator deflection and RCS torque
        qdyn = 0.5 * self.rho * self.x[3]**2  # dynamic pressure
        dCL_dAoA = self.coeff_deriv_longitudinal[0] * 180 / np.pi
        dCm_dAoA = self.coeff_deriv_longitudinal[1] * 180 / np.pi

        A[0, 1] = qdyn * self.S * self.c / self.Iyy * dCm_dAoA
        A[1, 0] = 1
        A[1, 1] = -qdyn * self.S / (self.m * self.x[3]) * dCL_dAoA * np.cos(self.sigma)**2

        B[0, 0] = 1 / self.Iyy  # Elevator deflection
        B[1, 1] = 1 / self.Iyy  # RCS torque about y-axis

        return A, B

    def lateral_control(self):
        A = np.zeros((4, 4))
        B = np.zeros((4, 4))  # Now B has four columns: aileron, rudder, RCS x, and RCS z

        cg = np.cos(self.x[5])
        tg = np.tan(self.x[5])
        sa = np.sin(self.AoA)
        ca = np.cos(self.AoA)
        cs = np.cos(self.sigma)

        qdyn = 0.5 * self.rho * self.x[3]**2  # dynamic pressure
        g = self.mu / self.x[0]**2  # gravitational parameter
        L = qdyn * self.CL * self.S

        dCl_dAoS = self.coeff_deriv_lateral[3] * 180 / np.pi
        dCl_da = self.coeff_deriv_lateral[4]
        dCn_dAoS = self.coeff_deriv_lateral[5] * 180 / np.pi
        dCn_da = self.coeff_deriv_lateral[6]
        dCn_dr = self.coeff_deriv_lateral[7]

        A[0, 2] = qdyn * self.S * self.b / self.Ixx * dCl_dAoS
        A[1, 2] = qdyn * self.S * self.b / self.Izz * dCn_dAoS
        A[2, 0] = sa
        A[2, 1] = -ca
        A[2, 3] = (self.x[3] / self.x[0] - g / self.x[3]) * cg * cs
        A[3, 0] = -ca
        A[3, 1] = -sa
        A[3, 2] = -(L / (self.m * self.x[3]) * cs + (self.x[3] / self.x[0] - g / self.x[3]) * cg) * cs
        A[3, 3] = L / (self.m * self.x[3]) * tg * cs

        B[0, 0] = 1 / self.Ixx  # Aileron deflection
        B[1, 1] = 1 / self.Izz  # Rudder deflection
        B[2, 2] = 1 / self.Ixx  # RCS torque about x-axis
        B[3, 3] = 1 / self.Izz  # RCS torque about z-axis

        return A, B

    def compute_lqr(self, A, B):
        X = solve_continuous_are(A, B, self.Q_long, self.R_long)
        K = np.linalg.inv(self.R_long) @ B.T @ X
        return K

    def compute_thruster_commands_based_on_directions(self, force):
        # Create the thruster matrix using direction vectors only
        thruster_matrix = np.vstack((self.V.T, np.zeros((3, 4))))

        # Calculate the pseudo-inverse of the thruster matrix
        thruster_matrix_inv = np.linalg.pinv(thruster_matrix[:3, :])

        # Calculate thruster commands
        thruster_commands = thruster_matrix_inv @ force

        # Apply saturation limits
        thruster_commands = np.clip(thruster_commands, -self.thruster_limit, self.thruster_limit)

        return thruster_commands

    def compute_control_inputs(self):
        u_long = -self.K_long @ self.x[:2]  # Longitudinal control input
        u_lat = -self.K_lat @ self.x[:4]    # Lateral control input

        force_long = u_long[0]
        torque_long = u_long[1]
        force_lat = u_lat[:2]
        torque_lat = u_lat[2:]

        force = np.array([force_long, force_lat[0], force_lat[1]])
        torque = np.array([torque_lat[0], torque_long, torque_lat[1]])

        return force, torque

    def control(self):
        force, torque = self.compute_control_inputs()
        thruster_commands = self.compute_thruster_commands_based_on_directions(force)
        return thruster_commands

# Example usage
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

x = np.array([120000, 0, 0, 2000, 0, 0, 0, 0, 0, 10, 10, 10])
sigma = 75
coeff_deriv_longitudinal = [0.1, 0.01]
coeff_deriv_lateral = [0.1, 0.01, 0.01, 0.1, 0.01, 0.1, 0.01, 0.01]
rho = 0.02
S = 15
c = 3
b = 5
Iyy = 250
Ixx = 250
Izz = 250
m = 1000
mu = 4.282837e13
AoA = 0.1
CL = 0.5

control_system = ControlSystem(P_new, V, x, sigma, coeff_deriv_longitudinal, coeff_deriv_lateral,
                               rho, S, c, b, Iyy, Ixx, Izz, m, mu, AoA, CL)

thruster_commands = control_system.control()
print("Thruster Commands:", thruster_commands)



