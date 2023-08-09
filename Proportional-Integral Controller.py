# -*- coding: utf-8 -*-
"""
Created on Tue Aug  8 14:28:50 2023

@author: mrm
"""


#Proportional-Integral Controller
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

Kc = 1.7 #Controller gain
Tc = 4 #Reset time

PV0 = 20 
OP0 = 10
SP = np.ones(number_element)*20
SP[30:60] = 15
SP[60:] = 25
PV = np.empty(number_element)
P = np.empty(number_element)

OP = np.empty(number_element)
OP[0] = OP0 #make it bumpless
PV[0] = PV0 #make it bumpless
error = np.ones(number_element)

I = np.zeros(number_element)
I[0] = 0 #(SP[0] - PV[0])*delta_t

def level_controller():
    for i in range(0,number_element-1):   
        error[i] = SP[i] - PV[i]
        P[i] = Kc*error[i]
        OP[i] = OP0 + P[i]
        if i >= 1:
            #I[i] = I[i-1] + (error[i-1] + error[i])*delta_t     #Alternative 1 will accumulate area of error faster
            #I[i] = I[i-1] + (error[i-1] + error[i])/2*delta_t   #Alternative 2 will have higher area of error
            I[i] = I[i-1] + error[i]*delta_t
            OP[i] = OP0 + P[i] + Kc/Tc*I[i]
        tspan = [delta_t*t[i],delta_t*t[i+1]]
        y = odeint(level_dynamic,PV[i],tspan,args=(OP[i],))
        PV[i+1] = y[-1]
        P[number_element-1] = P[number_element-2] 
        I[number_element-1] = I[number_element-2] 
        OP[number_element-1] = OP[number_element-2] 
    return (PV,OP)

(PV,OP) = level_controller()

plt.figure(1)
plt.plot(t,PV,'b',label='level(t)',linewidth=3)
plt.plot(t,SP,'k--',label='level setpoint(t)',linewidth=1)
plt.xlabel('time')
plt.ylabel('level, meter')
plt.xlim(20,75)
plt.legend()

plt.figure(2)
plt.plot(t,OP,'m',label='OP(t)',linewidth=3)
plt.xlabel('time')
plt.ylabel('OP, m^3/hours')
plt.xlim(20,75)
plt.legend()