# -*- coding: utf-8 -*-
"""
Created on Tue Aug  8 14:28:50 2023

@author: mrm
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

def level_dynamic(h,t,u):
    A = 2                       #Tank area, m^2
    Rv = 0.5                    #flow resistance, m^2/min
    dhdt = 1/A*u - Rv/A*h
    return dhdt
# time series
nt = 2400
t = np.linspace(0,nt/5,nt)
dt = t[1]-t[0]
# controller
Kc = 1.2     
Tc = 1.5                        
# setpoint
SP = np.ones(nt)*60
SP[150:] = 65
# process variable
PV = np.empty(nt)
PV0 = 60
PV[0] = PV0
# controller output
OP = np.empty(nt)
OP0 = 60
OP[0] = OP0
# controller
error = np.empty(nt)
error[0] = 0
P = np.empty(nt)
I = np.zeros(nt)

def level_controller():
    for i in range(0,nt-1):
        # controller
        error[i] = SP[i] - PV[i]
        P[i] = Kc*error[i]
        if i >= 1:
            I[i] = I[i-1] + error[i]*dt
        OP[i] = OP0 + P[i] + Kc/Tc*I[i]
        flowmax = 10
        u = OP[i]/100*flowmax
        # process
        y = odeint(level_dynamic,PV[i]/100*5,[0,dt],args=(u,))
        # sensor
        PV[i+1] = y[1,0]
        PV[i+1] = PV[i+1]/5*100
        P[nt-1] = P[nt-2] 
        I[nt-1] = I[nt-2] 
        OP[nt-1] = OP[nt-2] 
    return (PV,OP)

(PV,OP) = level_controller()

fig, ax = plt.subplots(2,1,layout = 'constrained')
ax[0].plot(t, PV, 'b',label='level(t)',linewidth=1)
ax[0].plot(t,SP,'k--',label='level setpoint(t)',linewidth=1)
ax[0].set_ylabel('level, %')
major_xticks = np.arange(0,nt/5,10)
minor_xticks = np.arange(0,nt/5,1)
ax[0].set_xticks(major_xticks)
ax[0].set_xticks(minor_xticks, minor = True)
ax[0].grid(True)
ax[0].text(44.87,67.1, 'K=1.2 T=1.5', horizontalalignment='center', \
verticalalignment='center')
ax[0].set_xlim(20,90)
ax[0].set_ylim(55,70)
ax[0].legend()


ax[1].set_xticks(major_xticks)
ax[1].set_xticks(minor_xticks, minor = True)
ax[1].grid(True)
ax[1].plot(t,OP,'m',label='OP(t)',linewidth=1)
ax[1].set_xlabel('time')
ax[1].set_ylabel('OP, %')
ax[1].set_xlim(20,90)
ax[1].set_ylim(10,40)
ax[1].legend()
plt.show()
