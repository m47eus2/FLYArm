import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from scipy import linalg

#
# Simulation
#

mb = 0.0335
l = 0.128
ms = 0.02335
xs = 0.120
b = 0.00011
g = 9.81

I = (1/3)*mb*l**2 + ms*xs**2
I = I/8.8

ku = 0.001617342

Fs = 100
# Składnik tarcia suchego
# - (Fc/I)*np.tanh(x[1]/0.01)
# Fc = 0.0006

def deg(t,x):
    dx1 = x[1]
    dx2 = -((mb*g*(l/2) + ms*g*xs)/I)*np.sin(x[0]) - (b/I)*x[1] + (ku*Fs*xs)/I
    return [dx1,dx2]

res = solve_ivp(deg, [0,10], [0,0], rtol=1e-10, atol=1e-10)

#
# Linearization
#

A = np.array([[0,1],
              [0,-b/I]])
B = np.array([[0],
              [ku*xs/I]])

#
# LQR regulator 
#

Q = np.eye(2)*1000
R = 1
P = linalg.solve_continuous_are(A,B,Q,R)
K = (1/R)*(B.T @ P)
print(K)

#
# LQR simulation
#

def degLQR(t,x):
    Fs = -K @ (x - [np.pi/2,0]) + 250
    Fs = Fs.item()

    dx1 = x[1]
    dx2 = -((mb*g*(l/2) + ms*g*xs)/I)*np.sin(x[0]) - (b/I)*x[1] + (ku*Fs*xs)/I
    return [dx1,dx2]

resLQR = solve_ivp(degLQR, [0,3], [0,0], rtol=1e-10, atol=1e-10)

x1 = resLQR.y[0]
x2 = resLQR.y[1]
x = np.vstack((x1,x2))
x0 = np.array([[np.pi/2],[0]])
x_tilde = x - x0
u = -K @ x_tilde + 250
u = u.flatten()

# plt.title("Odpowiedź wahadła")
# plt.plot(res.t, res.y[0], label="Simulation")
# #plt.plot(res.t, res.y[1], label="x2")
# plt.legend()
# plt.grid()
# plt.show()
plt.figure()
plt.title("Układ z regulatorem LQR")
plt.plot(resLQR.t, resLQR.y[0])
plt.grid()

plt.figure()
plt.title("Sterowanie w układzie z regulatorem LQR")
plt.plot(resLQR.t, u)
plt.grid()
plt.show()