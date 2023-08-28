# -*- coding: utf-8 -*-
"""
Created on Wed Aug 16 10:38:25 2023

@author: M.R.M
"""

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# Cascade controller for level controller of a liquid tank



def LevelResponse(level,time,ValveLift,delP,FlowOut,Physical_Parameter):
    
    """
    level : level of the liquid inside the tank
    time : time, hour
    ValveLift : Percent opening of inlet valve, 0 - 100 %
    delP : pressure difference in the input flow as disturbance variation
    FlowOut : As disturbance also but for this case, it's constant
    """
    rho = Physical_Parameter.rho
    Cv = Physical_Parameter.Cv
    sg = Physical_Parameter.sg
    A = Physical_Parameter.A
    
    FlowIn = rho*Cv*ValveLift*np.sqrt(delP/sg)
    FlowLeak = 5*level
    if level <0:
        dleveldt = 0
    else:
        dleveldt = (FlowIn - FlowOut - FlowLeak)/rho/A
    return dleveldt

#-------------------------------------Controller Engine----------------------------------------#
# time basis
num_index = 3001
t = np.linspace(0,num_index-1,num_index)
delta_t = t[1] - t[0]
rho = Physical_Parameter.rho
Cv = Physical_Parameter.Cv
sg = Physical_Parameter.sg
Kc_1 = PrimaryTuning.Kc
tauC_1 = PrimaryTuning.tauC
tauD_1 = PrimaryTuning.tauD
Kc_2 = SecondaryTuning.Kc
tauC_2 = SecondaryTuning.tauC
tauD_2 = SecondaryTuning.tauD

# storage
secondaryPV = np.empty(num_index)
secondaryOP = np.empty(num_index)
secondaryOP[0] = 30

primaryOP = np.zeros(num_index)
delP = np.empty(num_index)
FlowOut = np.empty(num_index)
    
for i in range(0,num_index-1):
    # primary/master controller
    # Apply disturbance to the system
    delP[i] = 12
    FlowOut[i] = 2
    secondaryPV[i] = rho*Cv*secondaryOP[i]*np.sqrt(delP[i]/sg)
    primaryerror = primarySP - primaryPV
    # if i >= 1:
    primaryioerror = primaryioerror + primaryerror*delta_t
    # primarydpv[i] = (primaryPV[i] - primaryPV[i-1])/delta_t
    primaryP = Kc_1*primaryerror
    primaryI = Kc_1/tauC_1*primaryioerror
    # primaryD[i] = Kc_1/tauD_1*primarydpv[i]
    primaryOP[i] = primaryOP0 + primaryP + primaryI # + primaryD[i]
    # Anti-reset windup protection
    if primaryOP[i] > 40:
        primaryOP[i] = 40
        primaryioerror = primaryioerror - primaryerror*delta_t 
    elif primaryOP[i] < 0:
        primaryOP[i] = 0
        primaryioerror = primaryioerror + primaryerror*delta_t
            
    # Honeywell Regulatory Controller DCS. For this time, we'll just skip this.
        
        """ I've got the master output which then send to the slave controller its setpoint
        The output ranges from 0 - 100 %, then i need to convert this value to appropriate
        setpoint of slave controller. Let's say maximum flow is ... and highest flow is ...
        then this the setpoint would be primaryOP*(PVEUHI - PVEULO). This signal then send
        to slave controller"""
    
    # PVEUHI = 35
    # PVEULO = 0
    # VariableSpan = PVEUHI - PVEULO
    # secondarySP[i] = primaryOP[i]/100*VariableSpan
    # Transfer from master controller to slave controller
    secondarySP[i] = primaryOP[i]
    secondaryerror = secondarySP[i] - secondaryPV[i]
    # if i >= 1:
    secondaryioerror = secondaryioerror + secondaryerror*delta_t
    # secondarydpv[i] = (secondaryPV[i] - secondaryPV[i-1])/delta_t
    secondaryP = Kc_2*secondaryerror
    secondaryI = Kc_2/tauC_2*secondaryioerror
    # secondaryD[i] = Kc_2/tauD_2*secondarydpv[i]
        
    # secondaryOP[i] = secondaryOP0 + secondaryP + secondaryI + secondaryD
        
    secondaryOP[i+1] = secondaryOP[i] + secondaryP + secondaryI + secondaryD
        
    # Anti-reset windup protection
    if secondaryOP[i] > 100:
        secondaryOP[i] = 100
        secondaryioerror[i] = secondaryioerror[i-1] - secondaryerror[i]*delta_t 
    elif secondaryOP[i] < 0:
        secondaryOP[i] = 0
        secondaryioerror[i] = secondaryioerror[i-1] + secondaryerror[i]*delta_t
        
    # m_in[i] = rho*Cv*op[i]*np.sqrt(delP[i]/sg)
    # h = odeint(LevelResponse,level[i],[0,delta_t],args=(op[i],delP[i],m_out[i]))
    # level[i+1] = h[-1]

    # system dynamic
    h = odeint(LevelResponse,primaryPV[i],[0,delta_t],args=(secondaryOP[i],delP[i],FlowOut[i]))
    primaryPV[i+1] = h[-1]

    
plt.figure(1)
plt.subplot(2,1,1)
plt.plot(t,primaryPV,'b-',linewidth=3,label='level')
plt.ylabel('Tank Level')
plt.legend(loc='best')
plt.subplot(2,1,2)
plt.plot(t,secondaryOP,'r--',linewidth=3,label='valve')
plt.ylabel('Valve')    
plt.legend(loc=1)
plt.figure(2)
plt.subplot(2,1,2)
plt.plot(t,secondaryPV,'b--',linewidth=3,label='Inlet Flow (kg/sec)')
plt.plot(t,delP,'r:',linewidth=3,label='Inlet Pressure (bar)')
plt.ylabel('Disturbances')    
plt.legend(loc=1)
plt.subplot(2,1,2)
plt.plot(t,FlowOut,'k:',linewidth=3,label='Outlet Flow (kg/sec)')
plt.xlabel('Time (sec)')
plt.legend(loc=1)

plt.show()