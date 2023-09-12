# -*- coding: utf-8 -*-
"""
Created on Thu Aug  3 16:57:51 2023

@author: mrm
"""
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

def level_integrating_process(h,t,u):
    A = 1
    c = 50
    rho = 1000
    dhdt = c*u/rho/A
    #c is valve constant kg/s/%opening
    #A is tank area
    #rho is fluid density
    #u is %valve opening
    #h is level, m^3
    return dhdt

n_element = 200
t = np.linspace(0,n_element-1,n_element)
delta_t = t[1] - t[0]
PV0 = 6
OP0 = 0 #0 - 100%
Kc = 6
SP = np.ones(n_element)*10
PV = np.empty(n_element)
PV[0] = PV0
OP = np.empty(n_element)
OP[0] = OP0
error = np.empty(n_element)
error[0] = SP[0] - PV[0]

def level_controller():
    for i in range(1,n_element):
        OP[i] = OP0 + Kc*error[i-1]
        tspan = [delta_t*t[i-1],delta_t*t[i]]
        y = odeint(level_integrating_process,PV[i-1],tspan,args=(OP[i],))
        PV[i] = y[-1]
        error[i] = SP[i] - PV[i]
    return (PV, OP)

(PV,OP) = level_controller()

plt.figure(1)
plt.subplot(2,1,1)
plt.plot(t,PV,'b',linewidth=2,label='level(t)')
plt.plot(t,SP,'k--',linewidth=2,label='SP level(t)')
plt.xlabel('time')
plt.ylabel('level, m')
plt.legend()

plt.subplot(2,1,2)
plt.plot(t,OP,'m',linewidth=2,label='% Valve(t)')
plt.xlabel('time')
plt.ylabel('Valve, %')
plt.legend()

