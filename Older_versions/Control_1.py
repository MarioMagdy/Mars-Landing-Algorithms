import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from scipy.optimize import fmin_slsqp

# Define the ODE function
def edl_ode(x, t):
    # Extract state variables
    phi, theta, h, V, y, psi, sig = x

    # Mars Constants
    Rm = 3376.2 * 1000      # Mars Radius (m)
    mu = 4.2828 * 10**13     # Mars Gravitational Parameter (m^3/s^2)
    rho0 = 0.02              # Mars Sea-Level Density kg/m^3 
    H = 11.1                 # Mars scale height (km)

    # Reentry Vehicle Constants
    m = 800                  # Vehicle Mass (kg)
    Cd = 1.7                 # Vehicle Drag Coefficient
    L_D = 0.22               # Vehicle L/D ratio
    radius = 1.65            # Shell radius (m)
    S = np.pi * radius**2    # Shell Area (m^2)

    # Other parameters
    rho = rho0 * np.exp(-h / (H * 1000))  # Atmospheric Density (kg/m^3)
    D = (1 / 2) * rho * V**2 * S * Cd     # Drag Force
    L = L_D * D                            # Lift Force
    M = V / np.sqrt(1.294 * 188.92 * 148.15)  # Mach Number

    r = Rm + h
    g = mu / (Rm + h)**2

    # Equations of motion
    phi_dot = (V / (Rm + h)) * np.cos(y) * np.sin(psi) / np.cos(theta)
    theta_dot = (V / (Rm + h)) * np.cos(y) * np.cos(psi)
    h_dot = V * np.sin(y)
    V_dot = -(D / m) - (g * np.sin(y))
    y_dot = (L / m / V * np.cos(sig)) + (np.cos(y) * ((V / (Rm + h)) - (g / V)))
    psi_dot = ((1 / m / V / np.cos(y)) * L * np.sin(sig)) + ((V / (Rm + h) / np.cos(theta)) * np.cos(y) * np.sin(psi) * np.sin(theta))
    # Bank angle equation
    sig_dot = ((1 / m / V / np.cos(y)) * L * np.sin(sig)) + ((V / (Rm + h) / np.cos(theta)) * np.cos(y) * np.sin(psi) * np.sin(theta))

    return [phi_dot, theta_dot, h_dot, V_dot, y_dot, psi_dot, sig_dot]

# Define the cost function for optimization
def objective_func(x):
    sim_time = x[0]
    NUM = 50
    delta_time = sim_time / NUM
    
    vels = x[3 * NUM + 1:4 * NUM + 1]
    acc = [abs((vels[i + 1] - vels[i]) / delta_time) for i in range(NUM - 1)]
    max_acc = max(acc)
    
    return max_acc

def constraint_func(x):
  sim_time = x[0]
  NUM = 50
  delta_time = sim_time / NUM

  # Check if bnks exist in the solution (avoiding potential IndexError)
  if len(x) >= 6 * NUM + 1:
    bnks = x[6 * NUM + 1:7 * NUM + 1]
  else:
    bnks = []  # Set bnks to empty list if not found

  c = [abs(bnks[i + 1] - bnks[i]) - 0.0872665 for i in range(len(bnks) - 1)]

  return c


# Main script
if __name__ == "__main__":
    # Define initial conditions
    state0 = np.array([-1.472993311, 0.5894954268, 125 * 1000, 7.3 * 1000, -15 * (np.pi / 180), 0 * (np.pi / 180), 0])
    sim_time = 510
    NUM = 50
    t = np.linspace(0, sim_time, NUM)

    # Integrate ODEs to simulate trajectory
    optimal = fmin_slsqp(objective_func, state0, f_eqcons=constraint_func)

    # Plotting
    lons = optimal[NUM + 1:2 * NUM + 1]
    lats = optimal[2 * NUM + 1:3 * NUM + 1]
    alts = optimal[3 * NUM + 1:4 * NUM + 1]
    # Extracting bank angles from the optimal array
    start_index = 6 * NUM + 1
    end_index = 7 * NUM + 1
    
    if len(optimal) >= end_index:
        bnks = optimal[start_index:end_index]
        # Print size and values of bnks 
        print("Size of bnks:", len(bnks))
        print("Values of bnks:", bnks)   
        # Plot bank angle 
        if len(bnks) > 0:
            # Plot bank angle
            fig2 = plt.figure(2)
            t = np.linspace(0, sim_time, NUM)
            plt.plot(t, np.degrees(bnks))
            plt.xlabel('time (sec)')
            plt.ylabel('Bank Angle (deg)')
            plt.grid(True)
            plt.axis('square')
            plt.show()
        else:
            print("No bank angle data available.")
    else:
        print("Not enough data in optimal array to extract bank angles.")
    
    # Plot trajectory
    fig1 = plt.figure(1)
    ax1 = fig1.add_subplot(111, projection='3d')
    ax1.scatter(np.degrees(lons), np.degrees(lats), alts / 1000, '.')
    ax1.set_xlabel('Longitude (deg)')
    ax1.set_ylabel('Latitude (deg)')
    ax1.set_zlabel('Altitude (km)')
    ax1.axis('square')

