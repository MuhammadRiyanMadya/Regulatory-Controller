# -*- coding: utf-8 -*-
"""
Created on Wed Aug  2 21:41:09 2023

@author: mrm
"""


#Proportional Controller
#h is level, m3
#OP is valve controller of input flowrate, for simplicity it directly gives value of m^3/hours
#Kc = controller proportional gain
#SP = setopoint of controller variable, from level_dynamic equation, SP is setpoint for level h
#PV = process variable, from level_dynamic equation PV is notation for h (level)
#OP = Controller output is the amout of input flowrate that have to be manipulate to get to the setpoint


import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint




def level_dynamic(h,t,OP):
    A = 2 #Tank area, m^2
    Rv = 0.5 #simplified output flow resistance, m^2/hours
    dhdt = (1/Rv*OP - h)/(A/Rv) #simplified dynamic equation
    return dhdt

number_element = 100
t = np.linspace(0,number_element-1,number_element)
delta_t = t[1]-t[0]


PV0 = 20
OP0 = 0
Kc = 3
SP = np.ones(number_element)*20
PV = np.empty(number_element)
OP = np.empty(number_element)
OP[0] = OP0 #make it bumpless
PV[0] = PV0 #make it bumpless
error = np.ones(number_element)
error[0] = SP[0] - PV[0]

def level_controller():
    for i in range(1,len(t)):   
        OP[i] = OP0 + Kc*error[i-1]
        tspan = [delta_t*t[i-1],delta_t*t[i]]
        y = odeint(level_dynamic,PV[i-1],tspan,args=(OP[i],))
        PV[i] = y[-1]
        error[i] = SP[i] - PV[i]
    return (PV,OP)

(PV,OP) = level_controller()

plt.figure(1)
plt.subplot(2,1,1)
plt.plot(t,PV,'b',label='level(t)',linewidth=3)
plt.xlabel('time')
plt.ylabel('level, meter')
plt.xlim(0,20)
plt.legend()
plt.subplot(2,1,2)
plt.plot(t,OP,'m',label='OP(t)',linewidth=3)
plt.xlabel('time')
plt.ylabel('OP, m^3/hours')
plt.xlim(0,20)
plt.legend()