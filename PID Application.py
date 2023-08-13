# -*- coding: utf-8 -*-
"""
Created on Fri Aug 11 20:14:02 2023

@author: mrm
"""
import numpy as np
from scipy.integrate import odeint


class model(object):
    """
    System parameter
    ---------------
    system must be in the form of 'first order plus time delay or FOPTD'
    Kp      : process gain
    taup    : process time
    thetap  : process time delay
    
    """
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
    
    Kc = xc.Kc
    tauI = xc.tauI
    tauD = xc.tauD
    op_hi = xc.op_hi
    op_lo = xc.op_lo
    
    delta_t = t[1] - t[0]

    pv = np.empty(num_index)
    pv[0] = init_val
    sp = np.empty(num_index)
    error = np.empty(num_index) 
    ie = np.empty(num_index)
    dpv = np.empty(num_index)
    P = np.empty(num_index)
    I = np.empty(num_index)
    D = np.empty(num_index)
    op = np.empty(num_index)

    if mode == 0:
        op[100] = 2
        
    for i in range(0,num_index):
        # PID engine
        error[i] = sp[i] - pv[i]
        if i >= 1:
            dpv = (pv[i] - pv[i-1])/delta_t
            ioe[i] = ioe[i-1] + error[i]*delta_t
        P[i] = Kc*error[i]
        I[i] = Kc/tauI*ioe[i]
        D[i] = Kc/tauD*dpv[i]
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

        
        
