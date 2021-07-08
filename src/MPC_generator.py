from casadi import *

T = 4.                                  # Time horizon
N = 20                                  # Number of control intervals (sampling period = T/N)

# Declare model parameters              
g = 9.81                                # Gravitational acceleration
kRoll = .99                             # Attitude dynamics gains
kPitch = .94
kYaw = 1.
tauRoll = .15                           # Attitude dynamics time constants
tauPitch = .11
tauYaw = .32
#Q = diag([6, 6, 1, 5, 5, 1])            # State penalty (velocity and attitude)
Q = diag([6, 6, 6, 5, 5, 1])
Qe = diag([30, 30, 6])                  # State penalty (relative position between drone and rendezvous point)
R = diag([1, 1, 1, 1])                  # Control penalty
QN = diag([17.4, 17.7, 8.1, 1.1, 1.2, 3.3, 0.4, 0.3, 0.1])
QN[0,3] = 2.0
QN[3,0] = 2.0
QN[0,7] = .6
QN[7,0] = .6
QN[1,4] = 2.2
QN[4,1] = 2.2
QN[1,6] = -0.8
QN[6,1] = -0.8
QN[2,5] = 2.4
QN[5,2] = 2.4
QN[3,7] = .4
QN[7,3] = .4
QN[4,6] = -.5
QN[6,4] = -.5

# Declare model variables
x = MX.sym('x', 9, 1)                   # State vector
u = MX.sym('u', 4, 1)                   # Control vector
s = MX.sym('set',3,1)                   # Setpoint final position (rendezvous position)

# Dynamic model
xdot = vertcat(x[3], \
        x[4], \
        x[5], \
        (cos(x[6])*sin(x[7])*cos(x[8])+sin(x[6])*sin(x[8]))*((g+u[0])/(cos(x[6])*cos(x[7]))), \
        (cos(x[6])*sin(x[7])*sin(x[8])-sin(x[6])*cos(x[8]))*((g+u[0])/(cos(x[6])*cos(x[7]))), \
        u[0], \
        -x[6]/tauRoll  + kRoll/tauRoll*u[1], \
        -x[7]/tauPitch + kPitch/tauPitch*u[2], \
        -x[8]/tauYaw   + kYaw/tauYaw*u[3])

# Quadratic cost function
x_pos = vertcat(x[0], x[1], x[2])
x_velAtt = vertcat(x[3], x[4], x[5], x[6], x[7], x[8])
e = x_pos - s
cf = transpose(e) @ Qe @ e + transpose(x_velAtt) @ Q @ x_velAtt + transpose(u) @ R @ u

f  = Function('f', [x, u, s], [xdot, cf])  # Casadi function retrieving dynamics and cost value
fs = Function('f', [x, u], [xdot])  # Casadi function retrieving dynamics only

# Integrator (Fixed step Runge-Kutta 4)
Xi = MX.sym('Xi', 9, 1)
Ui = MX.sym('Ui', 4, 1)
DT = MX.sym('DT', 1, 1)
Xset = MX.sym('Xset', 3, 1)
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
rk4 = Function('rk4', [Xi, Ui, DT], [Xfs], ['x0','u','dt'],['xf'])
rk4.generate('rk4')
print(rk4)

# Optimization
opti = casadi.Opti()                    # Optimization environment initialization

x = opti.variable(9,N+1)                # Optimization variables
u = opti.variable(4,N)
p = opti.parameter(9,1)                 # Optimization parameter (initial state x0)
s = opti.parameter(3,1)                 # Optimization parameter (desired final state)

cost = 0                                # Cost initialization
opti.subject_to(x[:,0] == p)            # Fix initial state
for k in range(N):
    x_next, c = F(x[:,k],u[:,k],T/N,s)
    cost = cost + c
    opti.subject_to(x[:,k+1] == x_next) # Enforce dynamic model
XN = vertcat(x_next[0], x_next[1], x_next[2]) - s
XN = vertcat(XN, x_next[3], x_next[4], x_next[5], x_next[6], x_next[7], x_next[8])
cost = cost + transpose(XN) @ QN @ XN
opti.minimize(cost)                     # Optimization objective

px = x[0,:]                             # Horizontal x position
py = x[1,:]                             # Horizontal y position
pz = x[2,:]                             # Vertical position
vx = x[3,:]                             # x-velocity
vy = x[4,:]                             # y-velocity
vz = x[5,:]                             # Vertical velocity
roll = x[6,:]                           # Roll angle
pitch = x[7,:]                          # Pitch angle
yaw = x[8,:]                            # Yaw angle 
az_cmd = u[0,:]                         # Commanded vertical acceleration
roll_cmd = u[1,:]                       # Commanded roll angle
pitch_cmd = u[2,:]                      # Commanded pitch angle
yaw_cmd = u[3,:]                        # Commanded yaw angle

opti.subject_to(opti.bounded(-1.1,px,1.1))      # Enforce constraint on horizontal x position
opti.subject_to(opti.bounded(-1.5,py,1))        # Enforce constraint on horizontal y position
opti.subject_to(opti.bounded(0,pz,2))           # Enforce constraint on vertical position
opti.subject_to(opti.bounded(-2,vx,2))          # Enforce constraint on horizontal x velocity (replace constraint on max speed)
opti.subject_to(opti.bounded(-2,vy,2))          # Enforce constraint on horizontal y velocity (replace constraint on max speed)
#opti.subject_to(opti.bounded(-0.7,vz,0.5))      # Enforce constraint on vertical velocity
opti.subject_to(opti.bounded(-0.4,vz,0.5))
opti.subject_to(opti.bounded(-.5,roll,.5))      # Enforce constraint on roll angle
opti.subject_to(opti.bounded(-.5,pitch,.5))     # Enforce constraint on pitch angle

#opti.subject_to(opti.bounded(-1,az_cmd,1))      # Enforce constraint on commanded vertical acceleration
opti.subject_to(opti.bounded(-.5,az_cmd,.5))
opti.subject_to(opti.bounded(-.5,roll_cmd,.5))  # Enforce constraint on commanded roll angle
opti.subject_to(opti.bounded(-.5,pitch_cmd,.5)) # Enforce constraint on commanded pitch angle
                
for k in range(N):                      # Enforce a SIMPLIFIED constraint on maximum yaw rate (acting on yaw angle and commanded yaw angle)
    opti.subject_to(opti.bounded(-pi/12,(yaw_cmd[k]-yaw[k]),pi/12))

opts = {'qpsol': 'qrqp'}                # Setting the solver    
opti.solver('sqpmethod', opts)                  

# Code generation
MPC = opti.to_function('MPC', [p, s], [u[:,0]], ['x0', 'Setpoint'], ['controlAction'])
genOpts = {'with_header': False}
MPC.generate('MPC', genOpts)
print(MPC)