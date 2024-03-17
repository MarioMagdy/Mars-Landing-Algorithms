# ( this code has an error if you can solve it i'll give you a present )
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
    L_D = 0.24               # Vehicle L/D ratio
    radius = 1.65            # Shell radius (m)
    S = np.pi * radius**2    # Shell Area (m^2)
    Cd_pc = 1.1    # Vehicle Drag Coefficient Parachute
    S_pc = 107.2            #Parachute Area (m^2)
    # Other parameters
    rho = rho0 * np.exp(-h / (H * 1000))  # Atmospheric Density (kg/m^3)
    D = (1 / 2) * rho * V**2 * S * Cd     # Drag Force
    L = L_D*D                    # Lift Force
    L = L_D * D                            # Lift Force
    
    M = V / np.sqrt(1.294 * 188.92 * 148.15)  # Mach Number
    # Deploy Parachute at Mach
    if M <= 1.65:
        D = D + (0.5 * rho * V**2 * S_pc * Cd_pc)
        L = 0

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
    output_array = [phi, theta, h, V, y, psi, sig]
    
    return output_array
# Define the cost function for optimization
def objective_func(x):
    sim_time = float(x[0])  # Convert sim_time to float
    NUM = 50
    delta_time = sim_time / NUM
    # state vector x = [lon; lat; alt; vel; fpa; azi; bnk]
    lons = x[1:1 + NUM]
    lats = x[1 + NUM:1 + NUM * 2]
    alts = x[1 + NUM * 2:1 + NUM * 3]
    vels = x[1 + NUM * 3:1 + NUM * 4]
    fpas = x[1 + NUM * 4:1 + NUM * 5]
    azis = x[1 + NUM * 5:1 + NUM * 6]
    bnks = x[1 + NUM * 6:1 + NUM * 7]
    ind = np.arange(0, 49)
    acc = np.abs((vels[ind + 1] - vels[ind]) / delta_time)
    cost = np.max(acc)
    return cost

def constraint_func(x, t):
    # Extract simulation time and number of trajectory points
    sim_time = x[0]
    NUM = 50
    delta_time = sim_time / NUM
    
    # Check if the length of x is correct
    if len(x) != (7 * NUM + 1):
        raise ValueError("Input array 'x' has incorrect length")

    # Extract trajectory points from the input array x
    lons = x[1:NUM + 1]
    lats = x[NUM + 1: 2 * NUM + 1]
    alts = x[2 * NUM + 1: 3 * NUM + 1]
    vels = x[3 * NUM + 1: 4 * NUM + 1]
    fpas = x[4 * NUM + 1: 5 * NUM + 1]
    azis = x[5 * NUM + 1: 6 * NUM + 1]
    bnks = x[6 * NUM + 1: 7 * NUM + 1]

    # Check if any trajectory point arrays are empty
    if any(len(arr) == 0 for arr in [lons, lats, alts, vels, fpas, azis, bnks]):
        raise ValueError("One or more trajectory point arrays are empty")
        
    c = []
    ceq = []
    bnk_lim = 0.0872665
    ang_tol = 0.0174533 / 4
    tar_lat = 0.5894954268
    tar_lon = -1.472993311
    j = 0
    
    for i in range(len(lons) - 1):
        x_i = np.array([lons[i], lats[i], alts[i], vels[i], fpas[i], azis[i], bnks[i]])
        x_n = np.array([lons[i + 1], lats[i + 1], alts[i + 1], vels[i + 1], fpas[i + 1], azis[i + 1], bnks[i + 1]])
    
        xdot_i = edl_ode(x_i, t)  # Calculate derivative at current point
        xdot_n = edl_ode(x_n, t)  # Calculate derivative at next point
    
        print("delta_time:", delta_time)
        print("x_i:", x_i)
        print("xdot_n:", xdot_n)
        print("xdot_i:", xdot_i)
    
        # Ensure delta_time is a float
        delta_time = float(delta_time)
        print("Data type of delta_time:", type(delta_time))
        print("Data type of xdot_n:", type(xdot_n))
        print("Data type of xdot_i:", type(xdot_i))


        xdot_n = np.array(xdot_n[:-1])
        xdot_i = np.array(xdot_i[:-1])
        
        # Perform the element-wise addition and division
        xend = x_i[:-1] + delta_time * (xdot_n + xdot_i) / 2

        # Perform the multiplication
        #xend = x_i[:-1] + delta_time * (xdot_n + xdot_i) / 2
        #xend = x_i[:-1] + delta_time * (xdot_n + xdot_i) / 2
        ceq.extend(x_n[:-1] - xend)
        
        if j < 50:
            c.append(np.abs(bnks[j + 1] - bnks[j]) - bnk_lim)
        j += 1

    # Add additional constraints
    ceq.extend([np.abs(tar_lon - lons[-1]), np.abs(tar_lat - lats[-1]), np.abs(2 - alts[-1])])
    
    # Ensure all returned arrays have the same length as x0
    if len(c) != len(x) or len(ceq) != len(x):
        raise ValueError("Arrays returned by constraint_func have incompatible shapes")

    return c


# Main script
if __name__ == "__main__":
    # Define initial conditions
    state0 = np.array([-1.472993311, 0.5894954268, 125 * 1000, 7.3 * 1000, -15 * (np.pi / 180), 0 * (np.pi / 180), 0])
    NUM = 50
    sim_time = float(510)  # Convert sim_time to float
    delta_time = sim_time / NUM
    
    t = np.linspace(0, sim_time, NUM)
    # Construct the initial input array x
    # Construct the initial input array x0
    # Determine the length of x0 based on the lengths of lb and ub
   
    #Bounds: [sim_time, lon, lat, alt, vel, fpa, azi, bank]
    lb = np.array([100] + [-np.pi] * NUM + [-(np.pi/2)] * NUM + [0] * NUM + [100] * NUM + [-(np.pi/2)] * NUM + [-(np.pi/2)] * NUM + [-(np.pi/2)] * NUM)
    ub = np.array([2000] + [np.pi] * NUM + [(np.pi/2)] * NUM + [15e4] * NUM + [7000] * NUM + [(np.pi/2)] * NUM + [(np.pi/2)] * NUM + [(np.pi/2)] * NUM)
    # Perform the optimization with the corrected bounds
    #optimal = fmin_slsqp(objective_func, x0, bounds=bounds, f_eqcons=constraint_func, iter=100, acc=1e-06, iprint=- 1)
    # Print lengths for debugging
    # Check if sim_time and state0 are not array-like objects, convert them
    if not isinstance(sim_time, (list, np.ndarray)):
        sim_time = [sim_time]
    
    if not isinstance(state0, (list, np.ndarray)):
        state0 = [state0]
    
    # Determine the length of x0 based on the lengths of lb and ub
    x0_length = len(lb)
    # Create x0 with appropriate length
    x0 = np.hstack((sim_time, state0, np.zeros(x0_length - len(sim_time) - len(state0))))
 
    # Define your time parameter
    t = 0  # You need to set the appropriate value for 't'
    
    # Call fmin_slsqp with constraint_func and the 't' parameter
    optimal = fmin_slsqp(objective_func, x0, bounds=list(zip(lb, ub)), f_eqcons=lambda x: constraint_func(x, t), iter=100, acc=1e-06, iprint=-1)

    print("Length of lb:", len(lb))
    print("Length of ub:", len(ub))
    print("Length of x0:", len(x0))
   # optimal = fmin_slsqp(objective_func, x0, bounds=list(zip(lb, ub)), f_eqcons=constraint_func, iter=100, acc=1e-06, iprint=-1)
    
    if len(x0) != len(lb) or len(x0) != len(ub):
        print("Lengths of x0, lb, and ub do not match!")

    # Define options for the optimization
    options = {'ftol': 1e-6, 'maxiter': 1000, 'disp': True}
    # Integrate ODEs to simulate trajectory
    # Print the optimal solution
    print("Optimal solution:", optimal)

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

