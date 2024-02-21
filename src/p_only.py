# -*- coding: utf-8 -*-
"""
Created on Wed Aug  2 21:41:09 2023

@author: mrm
"""

#Proportional Controller
#h  = PV = tank level, meter
#OP = valve opening, m^3/hours
#Kc = proportional gain
#SP = setopoint, meter

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint




def level_dynamic(h,t,OP):
    A = 2                       #Tank area, m^2
    Rv = 0.5                    #flow resistance, m^2/hours
    Flowmax = 2.5               #flow max of the valve
    dhdt = Flowmax/A*OP/100 - h*Rv/A #dynamic equation
    return dhdt

number_element = 100
t = np.linspace(0,number_element-1,number_element)
delta_t = t[1]-t[0]


PV0 = 3
OP0 = 60
Kc = 13
SP = np.ones(number_element)*3
SP[20:60] = 4
SP[60:] = 2
PV = np.empty(number_element)
OP = np.empty(number_element)
OP[0] = OP0 #make it bumpless
PV[0] = PV0 #make it bumpless
error = np.empty(number_element)
error[0] = SP[0] - PV[0]

def level_controller():
    for i in range(1,len(t)):   
        OP[i] = OP0 + Kc*error[i-1]
        tspan = [delta_t*t[i-1],delta_t*t[i]]
        y = odeint(level_dynamic,PV[i-1],tspan,args=(OP[i],))
        PV[i] = y[1,0]
        error[i] = SP[i] - PV[i]
    return (PV,OP)

(PV,OP) = level_controller()

plt.figure(1)
plt.subplot(2,1,1)
plt.plot(t,PV,'b',label='level (t)',linewidth=3)
plt.plot(t,SP,'k--',label='setpoint(t)',linewidth=3)
plt.ylabel('level, meter')
plt.legend()
plt.subplot(2,1,2)
plt.plot(t,OP,'m',label='OP(t)',linewidth=3)
plt.xlabel('time')
plt.ylabel('OP, m^3/hours')
plt.legend()
plt.show()
