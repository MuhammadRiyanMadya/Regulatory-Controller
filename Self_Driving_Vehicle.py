# -*- coding: utf-8 -*-
"""
Created on Wed Aug  9 17:07:06 2023

@author: mrm
"""

#1 CAR MOMENTUM MODEL ESTIMATION

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

num_index = 120
t = np.linspace(0,num_index-1,num_index)
delta_t = t[1]-t[0]

def momentum_balance(v,t,u):
    m = 700 #kg (car body + passengers)
    Cd = 0.24 #dimensionless, drag coefficient
    rho = 1.225 #kg/m^3, air density
    A = 5 #m^2, cross-sectional area
    Fp = 30 #N/% pedal, thrust parameter
    dvdt = (Fp*u - 0.5*rho*A*Cd*v**2)/m #Non-linear ODEs
    return dvdt

vt = np.empty_like(t)
vt[0] = 0
u = np.zeros(num_index)
u[10:] = 50

for i in range(num_index-1):
    tstep = [t[i],t[i+1]]
    v = odeint(momentum_balance,vt[i],tstep,args=(u[i],))
    vt[i+1] = v[-1]
    vt[num_index-1] = vt[num_index-2]

#plt.figure(1)
#plt.subplot(2,1,1)
#plt.plot(t,vt,'b',linewidth = 2,label='velocity(t)')
#plt.legend()
#plt.subplot(2,1,2)
#plt.plot(t,u,'r--',linewidth = 2,label='% pedal')
#plt.legend()

#2 FOPTD ESTIMATION FROM TRANSIENT RESPONSE

uf = interp1d(t,u)
def foptd(y,t,uf,Kp,taup,thetap):
    try:
        if (t-thetap) <= 0 :
            um = uf(0)
        else:
            um = uf(t-thetap)
    except:
        um = u[0]
    dydt = (Kp*(um-u[0]) - (y-vt[0]))/taup
    #dydt = (Kp*um - y)/taup
    return dydt

def foptd_solver(x):
    Kp = x[0]
    taup = x[1]
    thetap = x[2]
    yt = np.zeros(num_index)
    yt[0] = 0 #vt[0]
    for i in range(num_index-1):
        tstep = [t[i],t[i+1]]
        y = odeint(foptd,yt[i],tstep,args=(uf,Kp,taup,thetap)) #why if i put uf(i) the function is not working
        yt[i+1] = y[-1]
        yt[num_index-1] = yt[num_index-2]    
    return yt

x0 = [1,5,15]

yt = foptd_solver(x0)

plt.figure(1)
plt.subplot(2,1,1)
plt.plot(t,vt,'b',linewidth = 2,label='Velocity(t)')
plt.plot(t,yt,'r--',linewidth = 2,label='Foptd(t)')
plt.ylabel('Velocity, m/s')
plt.xlabel('Time, second')
plt.legend()
plt.subplot(2,1,2)
plt.plot(t,u,'k--',linewidth = 2,label='% pedal')
plt.ylabel('% Pedal')
plt.xlabel('Time, second')
plt.legend()


#3 FITTING OPTIMIZATION-GETTING USEFUL PARAMETERS FOR REGULATORY CONTROLLER


 