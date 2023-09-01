# -*- coding: utf-8 -*-
"""
Created on Wed Aug 16 10:38:25 2023

@author: M.R.M
"""

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

"""
This module simulates cascade controller of a level control system of a tank. This controller employs cascade strategy to reject the 
disturbances from inlet pressure as well as demand fluctuation from outlet flow. Both of master controler and slave controller have 
PID parameter and anti-reset windup protection. This system could be used to simulate real world controller and was created to be similar
to honeywell TDC 300 regulatory controller.
"""


class Physical_Parameter(object):
    rho = 1000
    Cv = 0.0001
    sg = 1
    A = 5
class PrimaryTuning(object):
    Kc_1 = 6.25
    tauC_1 = 60
    tauD_1 = 0
class SecondaryTuning(object):
    Kc_2 = 1.5
    tauC_2 = 1
    tauD_2 = 0
def LevelResponse(level,time,ValveLift,delP,FlowOut):
    
    """
    level         : level of the liquid inside the tank
    time          : time, hour
    ValveLift     : Percent opening of inlet valve, 0 - 100 %
    delP          : pressure difference in the input flow as disturbance variation
    FlowOut       : As disturbance also but for this case, it's constant
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

# Time basis
num_index = 3001
t = np.linspace(0,num_index-1,num_index)
delta_t = t[1] - t[0]

# Import constants
rho = Physical_Parameter.rho
Cv = Physical_Parameter.Cv
sg = Physical_Parameter.sg
# tauD_1 = PrimaryTuning.tauD
Kc_1 = PrimaryTuning.Kc_1
tauC_1 = PrimaryTuning.tauC_1
tauD_1 = PrimaryTuning.tauD_1
# tauD_2 = SecondaryTuning.tauD
Kc_2 = SecondaryTuning.Kc_2
tauC_2 = SecondaryTuning.tauC_2
tauD_2 = SecondaryTuning.tauD_2

# storage array
primaryPV = np.empty(num_index)
primaryPV[0] = 1
secondaryPV = np.empty(num_index)
secondaryOP = np.empty(num_index)
secondaryOP[0] = 30
primaryOP = np.zeros(num_index)
primaryOP[0] = 10.3923 
delP = np.empty(num_index)
FlowOut = np.empty(num_index)
FlowOut[0:] = 2

# derivative storage
primarydpv = np.empty(num_index)
primarydpv[0] = 0
primaryD = np.zeros(num_index)
secondarydpv = np.empty(num_index)
secondarydpv[0] = 0
secondaryD = np.zeros(num_index)

#disturbances storage
noise = np.empty(num_index)

# initial condition
primarySP = 1
primaryioerror = 0
secondaryioerror = 0


    
for i in range(0,num_index-1):
    
    # generate disturbances
    # noise[0:] = np.random.normal(0,1,num_index)
    # if i < num_index-1:
    delP[i] = 12 + np.sin(t[i]/100)
    stepseq = 0 + i*400
    if stepseq < num_index:
        if stepseq/400 % 2 != 0:
            FlowOut[stepseq:] = 3
        else: 
            FlowOut[stepseq:] = 2.7
    secondaryPV[i] = rho*Cv*secondaryOP[i]*np.sqrt(delP[i]/sg)

    # start the controller
    primaryerror = primarySP - primaryPV[i]
    if i > 1: 
        primaryioerror = primaryioerror + primaryerror*delta_t
        primarydpv[i] = (primaryPV[i] - primaryPV[i-1])/delta_t
    primaryP = Kc_1*primaryerror
    primaryI = Kc_1/tauC_1*primaryioerror
    primaryD[i] = Kc_1*tauD_1*primarydpv[i]
    primaryOP[i] = primaryOP[0] + primaryP + primaryI + primaryD[i]
    
    # Anti-reset windup protection
    if primaryOP[i] > 40:
        primaryOP[i] = 40
        primaryioerror = primaryioerror - primaryerror*delta_t 
    elif primaryOP[i] < 0:
        primaryOP[i] = 0
        primaryioerror = primaryioerror - primaryerror*delta_t
            
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
    
    secondarySP = primaryOP[i]
    secondaryerror = secondarySP - secondaryPV[i]
    if i > 1: 
        secondaryioerror = secondaryioerror + secondaryerror*delta_t
        secondarydpv[i] = (secondaryPV[i] - secondaryPV[i-1])/delta_t
    secondaryP = Kc_2*secondaryerror
    secondaryI = Kc_2/tauC_2*secondaryioerror
    secondaryD[i] = Kc_2*tauD_2*secondarydpv[i]
    secondaryOP[i+1] = secondaryOP[i] + secondaryP + secondaryI + secondaryD[i]
    
    # Alternative ways (without storage)
    # secondaryOP[i] = secondaryOP0 + secondaryP + secondaryI + secondaryD 
        
    # Anti-reset windup protection
    if secondaryOP[i] > 100:
        secondaryOP[i] = 100
        secondaryioerror = secondaryioerror - secondaryerror*delta_t 
    elif secondaryOP[i] < 0:
        secondaryOP[i] = 0
        secondaryioerror = secondaryioerror - secondaryerror*delta_t
        
    # old script
    # m_in[i] = rho*Cv*op[i]*np.sqrt(delP[i]/sg)
    # h = odeint(LevelResponse,level[i],[0,delta_t],args=(op[i],delP[i],m_out[i]))
    # level[i+1] = h[-1]

    # system dynamic
    level = odeint(LevelResponse,primaryPV[i],[0,delta_t],args=(secondaryOP[i],delP[i],FlowOut[i]))
    primaryPV[i+1] = level[-1]

# Normalize end value
secondaryPV[num_index-1] = secondaryPV[num_index-2]
delP[num_index-1] = delP[num_index-2]

# Visualization  
plt.figure(1)
plt.subplot(2,1,1)
plt.plot(t,primaryPV,'b-',linewidth=3,label='PV: level')
plt.axhline(y = 1,color = 'r',linestyle = '--',linewidth=2, label = 'SP: level')
plt.ylabel('Tank Level, meter')
plt.legend(loc='best')
plt.subplot(2,1,2)
plt.plot(t,secondaryOP,'r--',linewidth=3,label='OP: valve')
plt.ylabel('Valve, %')    
plt.legend(loc=1)
plt.figure(2)
plt.subplot(2,1,1)
plt.plot(t,delP,'r:',linewidth=3,label='Inlet Pressure (bar)')  
plt.ylabel('Pressure Noise, bar')
plt.legend(loc=1)
plt.subplot(2,1,2)
plt.plot(t,secondaryPV,'b--',linewidth=3,label='Inlet Flow (kg/sec)')
plt.ylabel('Inlet Fluctuation, m^3/h')   
plt.legend(loc=1)
plt.figure(3) 
plt.subplot(2,1,1)
plt.plot(t,FlowOut,'k:',linewidth=3,label='Outlet Flow (kg/sec)')
plt.xlabel('Time (sec)')
plt.ylabel('Deman Fluctuation, m^3/h')
plt.legend(loc=1)

plt.show()
