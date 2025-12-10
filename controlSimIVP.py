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

xd = np.pi/2
kp = 25
ki = 8
kd = 3.5

def model(t,X):
    x1, x2, intE = X

    e = xd - x1
    de = -x2 # d/dt (xd - x1) = -dx1/dt = -x2
    dIntE = e
    u = kp*e + ki*intE + kd*de

    if u>0: Fs=u
    else: Fs=0

    dx1 = x2
    dx2 = (-(mb*g*l/2 + ms*g*xs)/I)*np.sin(x1) - (b/I)*x2 + (Fs*xs)/I

    return [dx1,dx2,dIntE]

res = solve_ivp(model, [0,10], [0,0,0], rtol=1e-10, atol=1e-10)

plt.title("Stan układu")
plt.plot(res.t, res.y[0], label="x1")
#plt.plot(tList, x2List, label="x2")
#plt.plot(tList, uList, label="u")
plt.legend()
plt.grid()
plt.show()