import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

mb = 0.3
l = 0.3
ms = 0.02
xs = 0.25
b = 0.01
g = 9.81

I = (1/3)*mb*l**2 + ms*xs**2

xd = 1
kp = 1
ki = 3
kd = 3

def deg(t,x):
    e = xd - x[0]
    ie = x[2] # Integral of e
    de = e - x[3] # Last e

    Fs = kp*e + ki*ie + kd*de
    if Fs<0: Fs=0

    dx1 = x[1]
    dx2 = ((-mb*g*l/2 + ms*g*xs)/I)*np.sin(x[0]) - (b/I)*x[1] + (Fs*xs)/I

    dx3 = e
    dx4 = de
    return [dx1,dx2,dx3,dx4]

res = solve_ivp(deg, [0,10], [0,0,0,0], rtol=1e-10, atol=1e-10)

plt.plot(res.t, res.y[0], label="x1")
#plt.plot(res.t, res.y[1], label="x2")
plt.legend()
plt.grid()
plt.show()