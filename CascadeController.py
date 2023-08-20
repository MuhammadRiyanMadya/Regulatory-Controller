# -*- coding: utf-8 -*-
"""
Created on Wed Aug 16 10:38:25 2023

@author: mrm
"""

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

"Cascade controller for simple level controller of a liquid tank"

def LevelResponse(level,time,ValveLift,delP,FlowOut,Cv = 1e-4,rho = 1000,sg = 1, A = 5):
    """
    h : liquid level height inside tank
    t : time
    ValveLift : Percent opening of inlet valve, 0 - 100 %
    delP : pressure difference in the input flow as disturbance
    FlowOut : As disturbance also but for this case, it's constant
    """"
    
    FlowIn = rho*Cv*ValveLift*np.sqrt(delP/sg)
    FlowLeak = 5*h
    if level <0:
        dleveldt = 0
    else:
        dleveldt = (FlowIn - FlowOut - FlowLeak)/rho/A
    return dhdt

# time basis
num_index = 3001
t = np.linspace(0,num_index-1,num_index)
delta_t = t[1] - t[0]

# initial condition
primaryOP0 = 20
secondaryOP0 = .. 


primarySP = np.empty(num_index)
primarySP[0:] = 1
secondarySP = np.empty(num_index)

primaryPV = np.ones(num_index)
secondaryPV = np.ones(num_index)*7


secondaryOP = np.empty(num_index)
secondaryOP = np.empty(num_index)
secondaryOP[0:] = secondaryOP0

secondaryOP = np.empty(num_index)
secondaryOP = np.empty(num_index)
secondaryOP[0:] = secondaryOP0

primaryerror = np.empty(num_index)
primaryioerror = np.empty(num_index)
primarydpv = np.empty(num_index)
primaryP = np.empty(num_index)
primaryI = np.empty(num_index)
primaryD = np.empty(num_index)

secondaryerror = np.empty(num_index)
secondaryioerror = np.empty(num_index)
secondarydpv = np.empty(num_index)
secondaryP = np.empty(num_index)
secondaryI = np.empty(num_index)
secondaryD = np.empty(num_index)


# Disturbance
delP = np.empty(num_index)
delP[0:1000] = 12
delP[1000:] = 22

# Constant
m_out = np.empty(num_index)
m_out[0:] = 2




#---------------------------------------------------------------------------------------------#
for i in range(0,num_index-1):
    primaryerror[i] = primarySP[i] - primaryPV[i]
    if i == 0
    primaryOP[i] = OP0 - Kc*primaryerror[i]
    if i >= 1
        primaryioerror[i] = primaryioerror[i-1] + primaryerror[i]*delta_t
        primarydpv[i] = (primaryerror[i] - primaryerror[i-1])/delta_t
    primaryP[i] = Kc*error[i]
    primaryI[i] = Kc/tauC*primaryioerror[i]
    primaryD[i] = Kc/tauD*primarydpv[i]
    
    primaryOP[i] = primaryOP0 + Primary[i] + primaryI[i] + primaryD[i]
    if primaryOP[i] > 100:
        primaryOP[i] = 100
        primaryioerror[i] = primaryioerror[i-1] - primaryerror[i]*delta_t 
    else if primaryOP[i] < 0:
        primaryOP[i] = 0
        primaryioerror[i] = primaryioerror[i-1] + primaryerror[i]*delta_t
    #-----------------------------------------------------------------------------------------#
    """ I've got the master output which then send to the slave controller its setpoint
    The output ranges from 0 - 100 %, then i need to convert this value to appropriate
    setpoint of slave controller. Let's say maximum flow is ... and highest flow is ...
    then this the setpoint would be primaryOP*(PVEUHI - PVEULO). This signal then send
    to slave controller"""

    PVEUHI = 34
    PVEULO = 0
    range = PVEUHI - PVEULO
    secondarySP[i] = primaryOP[i]*range
    #------------------------------------------------------------------------------------------#
    secondaryerror[i] = secondarySP[i] - secondaryPV[i]
    if i == 0
    secondaryOP[i] = secondaryOP0 - Kc*secondaryerror[i]
    if i >= 1
        secondaryioerror[i] = secondaryioeerror[i-1] + secondaryerror[i]*delta_t
        secondarydpv[i] = (secondaryerror[i] - secondaryerror[i-1])/delta_t
    secondaruP[i] = Kc*secondaryerror[i]
    secondaryI[i] = Kc/tauC*secondaryioerror[i]
    secondaryD[i] = Kc/tauD*secondarydpv[i]
    
    secondaryOP[i] = OP0 + P[i] + I[i] + D[i]
    if secondaryOP[i] > 100:
        secondaryOP[i] = 100
        secondaryioerror[i] = secondaryioerror[i-1] - secondaryerror[i]*delta_t 
    else if primaryOP[i] < 0:
        secondaryOP[i] = 0
        secondaryioerror[i] = secondaryioerror[i-1] + secondaryerror[i]*delta_t
    
    # m_in[i] = rho*Cv*op[i]*np.sqrt(delP[i]/sg)
    # h = odeint(LevelResponse,level[i],[0,delta_t],args=(op[i],delP[i],m_out[i]))
    # level[i+1] = h[-1]
    
    seondaryPV[i] = rho*Cv*secondaryOP[i]*np.sqrt(delP[i]/sg)

    h = odeint(LevelResponse,primaryPV[i],[0,delta_t],args=(secondaryOP[i],delP[i],m_out[i]))
    primaryPV[i+1] = h[-1]
#-----------------------------------------------------------------------------------------------#
secondaryPV[num_index-1] = secondaryPV[num_index-2]

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
