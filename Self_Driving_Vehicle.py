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

num_index = 200
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

#plt.figure(1)
#plt.subplot(2,1,1)
#plt.plot(t,vt,'b',linewidth = 2,label='Velocity(t)')
#plt.plot(t,yt,'r--',linewidth = 2,label='Foptd(t)')
#plt.ylabel('Velocity, m/s')
#plt.xlabel('Time, second')
#plt.legend()
#plt.subplot(2,1,2)
#plt.plot(t,u,'k--',linewidth = 2,label='% pedal')
#plt.ylabel('% Pedal')
#plt.xlabel('Time, second')
#plt.legend()

#3 FITTING OPTIMIZATION-GETTING USEFUL PARAMETERS FOR REGULATORY CONTROLLER
def objective(x):
    yt = foptd_solver(x)
    SSE = 0
    for i in range(num_index):
        SSE = SSE + (vt[i] - yt[i])**2
    return SSE

x0 = [1,5,0]
SSE = objective(x0)
print('SSE Initial Value = ' + str(SSE))

solution = minimize(objective,x0)
x = solution.x
yt = foptd_solver(x)
SSE = objective(x)

print('SSE Final Value = ' + str(SSE))
print('Kp Predicted = ' + str(x[0]))
print('taup  Predicted = ' + str(x[1]))
print('thetap Predicted = ' + str(x[2]))


plt.figure(1)
plt.subplot(2,1,1)
plt.plot(t,vt,'b',linewidth = 3,label='Velocity(t)')
plt.plot(t,foptd_solver(x0),'y-.',linewidth = 1.5,label='Foptd Initial Value (t)')
plt.plot(t,yt,'m--',linewidth = 1.5,label='Foptd Optimized Value (t)')
plt.ylabel('Velocity, m/s')
plt.xlabel('Time, second')
plt.legend()
plt.subplot(2,1,2)
plt.plot(t,u,'k--',linewidth = 2,label='Pedal Step Test')
plt.ylabel('% Pedal')
plt.xlabel('Time, second')
plt.legend()

#4 PROCESS CONTROL-VELOCITY REGULATORY CONTROLLER
#value from final foptd parameters which are Kp, taup, thetap used to get
#best controller parameters including controller gain, reset time, and derivative value (PID)

def regulatory_controller(Kc,Tc)
    #value storage
    process_val = np.empty(num_index)
    process_val[0] = vt[0]
    process_set = np.empty(num_index)
    process_set[0:25] = 20
    process_set[70:120] = 100
    process_set[120:160] = 25
    OP = np.empty(num_index)
    OP[0] = u[0]
    error = np.empty(num_index)
    I = np.empty(num_index)
    for i in range(0,num_index-1):
        error[i] = process_set[i] - process_val[i]
        if i > 1:
            I[i] = I[i-1] + error[i]*delta_t
            OP[i] = OP[0] + Kc*error[i] + Kc/Tc*I[i]
        else:
            OP[i] = OP[0] + Kc*error[i]
        tstep = [t[i],t[i+1]]
        process_val = odeint(momentum_balance,process_val[i],tstep,args=(OP[i],Kc,Tc))
        process_val[i+1] = process_val[-1]
        process_val[num_index-1] = process_val[num_index-2]
    return process_val, setpoint

#4 PROCESS CONTROL-VELOCITY REGULATORY CONTROLLER
#value from final foptd parameters which are Kp, taup, thetap used to get
#best controller parameters including controller gain, reset time, and derivative value (PID)

def regulatory_controller(Kc,Tc):
    #value storage
    process_val = np.empty(num_index)
    process_val[0] = vt[0]
    process_set = np.empty(num_index)
    process_set[0:50] = 20
    process_set[50:120] = 100
    process_set[120:200] = 25
    OP = np.empty(num_index)
    OP[0] = u[0]
    error = np.empty(num_index)
    I = np.empty(num_index)
    for i in range(0,num_index-1):
        error[i] = process_set[i] - process_val[i]
        if i >= 1:
            I[i] = I[i-1] + error[i]*delta_t
            OP[i] = OP[0] + Kc*error[i] + Kc/Tc*I[i]
        else:
            OP[i] = OP[0] + Kc*error[i]
        tstep = [t[i],t[i+1]]
        process_val_tre = odeint(momentum_balance,process_val[i],tstep,args=(OP[i],))
        process_val[i+1] = process_val_tre[-1]
        OP[num_index-1] = OP[num_index-2]
        error[num_index-1] = error[num_index-2]
        I[num_index-1] = I[num_index-2]
        
    return process_val, process_set, OP

#Define Tc and Kc based on Kp, taup and thetap value
#For instance, Kc = 1/Kp and Tc = taup, while for time delay, I did not yet use it

(PV0, SP, OP0) = regulatory_controller(1/x0[0],x0[1]/2)
(PV, SP, OP) = regulatory_controller(1/x[0],x[1]/4)

plt.figure(2)
plt.subplot(2,1,1)
plt.plot(t,SP,'k--',linewidth = 1.5,label='Setpoint (t)')
plt.plot(t,PV0,'y--',linewidth = 2,label='Init ctrl (t)')
plt.plot(t,PV,'m--',linewidth = 3,label='Opt ctrl (t)')
plt.ylabel('Velocity, m/s')
plt.xlabel('Time, second')
plt.legend(loc='best')
plt.subplot(2,1,2)
plt.plot(t,OP0,'k--',linewidth = 1.5,label='% Pedal Init')
plt.plot(t,OP,'b:',linewidth = 2,label='% Pedal Opt')
plt.ylabel('% Pedal')
plt.xlabel('Time, second')
plt.legend(loc='best')


    
