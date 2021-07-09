from casadi import *

T = 4.                                  # Time horizon
N = 20                                  # Number of control intervals (sampling period = T/N)

# Declare model parameters
kvx = .94                               # Velocity dynamics     
tauvx = .08
kvy = .88                                   
tauvy = .08
Q = diag([1, 1])                        # State penalty (speed)
Qe = diag([30, 30])                     # State penalty (relative position between UGV and rendezvous point)
R = diag([1, 1])                        # Control penalty
P = diag([9.6, 9.9, 0.1, 0.1])          # Final state penalty
P[0,2] = 0.5
P[2,0] = 0.5
P[1,3] = 0.5
P[3,1] = 0.5

# Declare model variables
x = MX.sym('x', 4, 1)                   # State vector
u = MX.sym('u', 2, 1)                   # Control vector
s = MX.sym('set',2,1)                   # Setpoint final position (rendezvous position)

# Dynamic model
xdot = vertcat(x[2], x[3], -x[2]/tauvx  + kvx/tauvx*u[0], -x[3]/tauvy + kvy/tauvy*u[1])

# Quadratic cost function
x_pos = vertcat(x[0], x[1])
x_vel = vertcat(x[2], x[3])
e = x_pos - s
cf = transpose(e) @ Qe @ e + transpose(x_vel) @ Q @ x_vel + transpose(u) @ R @ u

f  = Function('f', [x, u, s], [xdot, cf])   # Casadi function retrieving dynamics and cost value
fs = Function('f', [x, u], [xdot])          # Casadi function retrieving dynamics only

# Integrator (Fixed step Runge-Kutta 4)
Xi = MX.sym('Xi', 4, 1)
Ui = MX.sym('Ui', 2, 1)
DT = MX.sym('DT', 1, 1)
Xset = MX.sym('Xset', 2, 1)
k1, k1_q = f(Xi, Ui, Xset)
k2, k2_q = f(Xi + DT/2 * k1, Ui, Xset)
k3, k3_q = f(Xi + DT/2 * k2, Ui, Xset)
k4, k4_q = f(Xi + DT * k3, Ui, Xset)
Xf       = Xi + DT/6 * (k1 + 2*k2 + 2*k3 + k4)
C        = DT/6 * (k1_q + 2*k2_q + 2*k3_q + k4_q)

F  = Function('F',[Xi, Ui, DT, Xset],[Xf,C],['x0','p','dt','set'],['xf','qf'])

# (Generate integrator for simulator node)
k1s = fs(Xi, Ui)
k2s = fs(Xi + DT/2 * k1s, Ui)
k3s = fs(Xi + DT/2 * k2s, Ui)
k4s = fs(Xi + DT * k3s, Ui)
Xfs = Xi + DT/6 * (k1s + 2*k2s + 2*k3s + k4s)
rk4_UGV = Function('rk4_UGV', [Xi, Ui, DT], [Xfs], ['x0','u','dt'],['xf'])
rk4_UGV.generate('rk4_UGV')
print(rk4_UGV)

# Optimization
opti = casadi.Opti()                    # Optimization environment initialization

x = opti.variable(4,N+1)                # Optimization variables
u = opti.variable(2,N)
p = opti.parameter(4,1)                 # Optimization parameter (initial state x0)
s = opti.parameter(2,1)                 # Optimization parameter (desired final state)

cost = 0                                # Cost initialization
opti.subject_to(x[:,0] == p)            # Fix initial state

for k in range(N-1):
    x_next, c = F(x[:,k],u[:,k],T/N,s)
    cost = cost + c
    opti.subject_to(x[:,k+1] == x_next) # Enforce dynamic model
x_end, c = F(x[:,N-1],u[:,N-1],T/N,s)   
XN = vertcat(x_end[0], x_end[1]) - s
XN = vertcat(XN, x_end[2], x_end[3])
opti.subject_to(x[:,N] == x_end)
cost = cost + transpose(XN) @ P @ XN
opti.minimize(cost)                     # Optimization objective

px = x[0,:]                             # x position
py = x[1,:]                             # y position
vx = x[2,:]                             # x velocity
vy = x[3,:]                             # y velocity 
vx_cmd = u[0,:]                         # Commanded x velocity
vy_cmd = u[1,:]                         # Commanded y velocity

opti.subject_to(opti.bounded(-1.1,px,1.1))      # Enforce constraint on x position
opti.subject_to(opti.bounded(-1.5,py,1))        # Enforce constraint on y position
opti.subject_to(opti.bounded(-0.3,vx,0.3))      # Enforce constraint on x velocity   (TODO: Better check max x and y velocity)   
opti.subject_to(opti.bounded(-0.2,vy,0.2))      # Enforce constraint on y velocity

opti.subject_to(opti.bounded(-0.3,vx_cmd,0.3))  # Enforce constraint on commanded x velocity (TODO: Update according to max vx and vy)
opti.subject_to(opti.bounded(-0.2,vy_cmd,0.2))  # Enforce constraint on commanded y velocity

opts = {'qpsol': 'qrqp'}                # Setting the solver    
opti.solver('sqpmethod', opts)  

# Code generation
MPC_UGV = opti.to_function('MPC_UGV', [p, s], [u[:,0]], ['x0', 'Setpoint'], ['controlAction'])
genOpts = {'with_header': False}
MPC_UGV.generate('MPC_UGV', genOpts)
print(MPC_UGV)
