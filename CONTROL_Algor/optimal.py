import numpy as np
from scipy.optimize import minimize

# Given data (example values, replace with actual data)
fx, fy, fz = 1.0, 2.0, 3.0
tx, ty, tz = 0.1, 0.2, 0.3

# Given data
force = np.array([fx, fy, fz])       # 3x1 matrix
torque = np.array([tx, ty, tz])      # 3x1 matrix
dir = np.random.rand(3, 4)          # 4x3 matrix
tor = np.random.rand(3, 4)          # 4x3 matrix

# Combine force and torque into a single vector
FT = np.hstack((force, torque)).flatten()  # 6x1 vector

# Combine dir and tor into a single matrix
D = np.vstack((dir, tor))             # 6x4 matrix

# Formulate the quadratic programming problem
H = np.eye(4)                # 14x14 identity matrix for minimizing the norm
f = np.zeros(4)              # 14x1 zero vector

# Equality constraints
Aeq = D
beq = FT

# Set lower bounds to ensure non-negativity
lb = np.zeros(4)          # Lower bounds (all zeros)
ub = np.inf * np.ones(4)  # No upper bounds

# Define the objective function for minimization
def objective(x):
    return 0.5 * np.dot(x, np.dot(H, x)) + np.dot(f, x)

# Define the equality constraint
def constraint_eq(x):
    return np.dot(Aeq, x) - beq

# Initial guess
x0 = np.zeros(4)

# Define the constraints as dictionaries
cons = ({'type': 'eq', 'fun': constraint_eq})

# Set the bounds
bounds = [(lb[i], ub[i]) for i in range(len(lb))]

# Solve the quadratic programming problem
solution = minimize(objective, x0, method='SLSQP', bounds=bounds, constraints=cons, options={'disp': True})

# Get the results
thrusters_cmd = solution.x
fval = solution.fun
exitflag = solution.success
output = solution

# Display the results
print('Thrusters Command:')
print(thrusters_cmd)
print('Norm of Thrusters Command:')
print(np.linalg.norm(thrusters_cmd))
print('Exit Flag:')
print(exitflag)
print('Output Information:')
print(output)


