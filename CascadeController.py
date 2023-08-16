# -*- coding: utf-8 -*-
"""
Created on Wed Aug 16 10:38:25 2023

@author: mrm
"""

import numpy as np

"Cascade controller for simple level controller of a liquid tank"

# Tank level transient response model


def LevelResponse(h,y,op,delP,mout):
    Cv = 1e-4 #
    rho = 1000 #kg/m^3
    sg = 1 #dimensionless
    A = 5 #m^2
    dhdt = (rho*Cv*op*np.sqrt(delP/sg) - mout)/rho/A
    return dhdt
