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

dt = 0.001
T = 10
N = int(T/dt)
x1 = np.pi/2
x2 = 0

tList = []
x1List = []
x2List = []
uList = []

kp = 1
ki = 0
kd = 0
xd = np.pi/2

integralE = 0
prevE = 0

for k in range(N):
    t = k*dt

    e = xd - x1
    integralE += e*dt
    u = kp*e + ki*integralE + kd*((e-prevE)/dt)
    prevE = e

    if u>0: Fs=u
    else: Fs=0
    
    dx1 = x2
    dx2 = (-(mb*g*l/2 + ms*g*xs)/I)*np.sin(x1) - (b/I)*x2 + (Fs*xs)/I

    x1 = x1 + dt*dx1
    x2 = x2 + dt*dx2

    tList.append(t)
    uList.append(Fs)
    x1List.append(x1)
    x2List.append(x2)

plt.title("Stan układu")
plt.plot(tList, x1List, label="x1")
#plt.plot(tList, x2List, label="x2")
#plt.plot(tList, uList, label="u")
plt.legend()
plt.grid()
plt.show()