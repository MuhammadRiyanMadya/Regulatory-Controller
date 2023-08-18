# -*- coding: utf-8 -*-
"""
Created on Wed Aug 16 10:38:25 2023

@author: mrm
"""

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

"Cascade controller for simple level controller of a liquid tank"

# Tank level transient response model


def LevelResponse(h,t,op,delP,m_out):
    m_in = rho*Cv*op*np.sqrt(delP/sg)
    m_leak = 5*h
    if h <0:
        dhdt = 0
    else:
        dhdt = (m_in - m_out - m_leak)/rho/A
    # op is valve lift from 0 - 100 %
    return dhdt

Cv = 1e-4 #
rho = 1000 #kg/m^3
sg = 1 #dimensionless
A = 5 #m^2


num_index = 3001
t = np.linspace(0,num_index-1,num_index)
delta_t = t[1] - t[0]

level = np.ones(num_index)

op = np.empty(num_index)
op[0:] = 20

delP = np.empty(num_index)
delP[0:1000] = 12
delP[1000:] = 22

m_out = np.empty(num_index)
m_out[0:] = 2

m_in = np.empty(num_index)


for i in range(0,num_index-1):
    primaryerror[i] = primarySP[i] - primaryPV[i]
    if i == 0
    primaryOP[i] = OP0 - Kc*error[i]
    if i >= 1
        primaryioerror[i] = primaryioerror[i-1] + primaryerror[i]*delta_t
        primarydpv[i] = (primaryerror[i] - primaryerror[i-1])/delta_t
    P = Kc*error[i]
    I = Kc/tauC*primaryioerror[i]
    D = Kc/tauD*primarydpv[i]
    
    primaryOP[i] = 
    


    
    m_in[i] = rho*Cv*op[i]*np.sqrt(delP[i]/sg)
    
    h = odeint(LevelResponse,level[i],[0,delta_t],args=(op[i],delP[i],m_out[i]))
    level[i+1] = h[-1]
    
m_in[num_index-1] = m_in[num_index-2]

plt.figure()
plt.subplot(4,1,1)
plt.plot(t,level,'b-',linewidth=3,label='level')
plt.ylabel('Tank Level')
plt.legend(loc='best')
plt.subplot(4,1,2)
plt.plot(t,op,'r--',linewidth=3,label='valve')
plt.ylabel('Valve')    
plt.legend(loc=1)
plt.subplot(4,1,3)
plt.plot(t,m_out,'b--',linewidth=3,label='Outlet Flow (kg/sec)')
plt.plot(t,delP,'r:',linewidth=3,label='Inlet Pressure (bar)')
plt.ylabel('Disturbances')    
plt.legend(loc=1)
plt.subplot(4,1,4)
plt.plot(t,m_in,'k:',linewidth=3,label='Inlet Flow (kg/sec)')
plt.xlabel('Time (sec)')
plt.legend(loc=1)

plt.show()
