# -*- coding: utf-8 -*-
"""
Created on Fri Aug 11 20:14:02 2023

@author: mrm
"""
import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt



class model(object):
    """
    System parameter
    ---------------
    system must be in the form of 'first order plus time delay or FOPTD'
    Kp      : process gain
    taup    : process time
    thetap  : process time delay
    
    """
    num_index = 100
    Kp = 2
    taup = 200
    thetap = 0
    init_val = 0
    
class pid(object):
    """
    Controller Input
    ----------------
    Kc = controller gain
    tauI = integral time
    tauD = derivative timw
    sp = setpoint
    op_hi = anti-reset windup high limit
    op_lo = anti-reset windup low limit

    """
    Kc = 2
    tauI = 10
    tauD = 0
    sp = []
    op_hi = 100
    op_lo = 20
    
def process(y,t,u,Kp,taup):
    dydt = -y/taup + Kp/taup*u
    return dydt
def calc_response(t,mode,xm,xc):
    """
    Parameters
    ----------
    t    : refer to the time duration input
    mode : auto or manual controller mode, auto = 1, manual = 0
    xm   : refer to the class of model system constants Kp, taup, thetap
    xc   : refer to the class of pid controller constants Kc, tauI, tauD
    
    """
    Kp = xm.Kp
    taup = xm.taup
    thetap = xm.thetap
    init_val = xm.init_val
    num_index = xm.num_index
    
    Kc = xc.Kc
    tauI = xc.tauI
    tauD = xc.tauD
    op_hi = xc.op_hi
    op_lo = xc.op_lo
    sp = xc.sp
    
    delta_t = t[1] - t[0]

    pv = np.empty(num_index+1)
    pv[0] = init_val
    error = np.empty(num_index+1) 
    ioe = np.empty(num_index+1)
    dpv = np.empty(num_index+1)
    P = np.empty(num_index+1)
    I = np.empty(num_index+1)
    D = np.empty(num_index+1)
    op = np.empty(num_index+1)

    if mode == 0:
        op[100:] = 2
        
    for i in range(0,num_index):
        # PID engine
        error[i] = sp[i] - pv[i]
        if i >= 1:
            dpv[i] = (pv[i] - pv[i-1])/delta_t
            ioe[i] = ioe[i-1] + error[i]*delta_t
        P[i] = Kc*error[i]
        I[i] = Kc/tauI*ioe[i]
        D[i] = -Kc*tauD*dpv[i]
        # Apply PID engine output onlye when mode is auto
        if mode == 1:
            op[i] = op[0] + P[i] + I[i] + D[i]
        # Equip auto mode with anti-reset windup to prevent error accumulation
        if op[i] > 100:
            op[i] = op_hi
            ioe[i] = ioe[i-1] - error[i]*delta_t
        if op[i] < 0:
            op[i] = op_lo
            ioe[i] = ioe[i-1] - error[i]*delta_t
        # simulate system with time delay as long as thetap
        # thetap is time delay of the process system, for instance it is 2.32 seconds then my
        # simulation duration is as long as num_index which is 2 menit or 120 seconds. I then apply
        # linspace to step with evenly spaced from start time to end time 0 - 120-th seconds. Let's say
        # my total step is 150. Then interval between one step to another step is 120/150 which is 4/5 or
        # 0.8 second. Then, the order of the time delay in the indices is 2.32/0.8 ~ 2.9-th.
        # So let's say 0, 0.8, 1.6, 2.4, 3.2. The exact time delay happens between 1.6 --> 2.4 or 2-th --> 3--th indices
        # Using numpy.ceil that 2.9-th index will turn to the ceil of that scalar which is smallest integer i where i > 2.9 
        # and it is 3.
        # in some cases, the method will experience deviation from reality. Consider the case where i have simulation duration
        # for 120 seconds, then i apply linspace of just 10 elements. Then the space between one step to the next step is 12 seconds
        # . The order of the time delay will be 2.32/12 = 0.1934 and by numpy.ceil function the time delay index will be 1-st index
        # which is 12 seconds after the error sample send to the controller. Not quite accurate of actual representation.
        ndelay = int(np.ceil(thetap/delta_t))
        iop = max(0,i-ndelay)
        y = odeint(process,pv[i],[0,delta_t],args=(op[iop],Kp,taup))
        pv[i+1] = y[-1]
    error[num_index] = error[num_index-1] 
    ioe[num_index] = ioe[num_index-1]
    dpv[num_index] = dpv[num_index-1]
    P[num_index] = P[num_index-1]
    I[num_index] = I[num_index-1]
    D[num_index] = D[num_index-1]
    op[num_index] = op[num_index-1]
    return (pv,op)



model.num_index = 1200
model.Kp = 2
model.taup = 200
model.thetap = 0
model.init_val = 0

t = np.linspace(0,model.num_index,model.num_index+1)

sp = np.zeros(model.num_index+1)  # set point
sp[50:600] = 10
sp[600:] = 0
pid.sp = sp

pid.Kc = 2
pid.tauI = 10
pid.tauD = 0
pid.op_hi = 100
pid.op_lo = 0

mode = 1

(pv,op) = calc_response(t,mode,model,pid)

plt.plot(t,pv)
plt.plot(t,sp)
        
