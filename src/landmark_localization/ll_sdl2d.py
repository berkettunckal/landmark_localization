#!/usr/bin/env python
# coding: utf-8

'''
2D Sub Defenite Localization plugin
'''

import landmark_localization.landmark_localization_core as llc
import numpy as np
from landmark_localization.sub_def_variable import sd_var
from landmark_localization.sub_def_multi_interval import sd_mi
from landmark_localization.sub_def_model import SDM, u_sqrt, u_atan2, u_tan, u_sin, u_cos, u_norm_angle, u_arcsin


# CONSTRANTS AND CORRECIONS FUNCTIONS

'''
Single Landmark R functions
F: (*x - xo)^2 + (*y - yo)^2 = (R + *dR)^2
'''

# f1: *x = sqrt((R + *dR)^2 - (*y - yo)^2) + Xo
def landmark_r_constrant_1(var, R, xo, yo, dr):
    return u_sqrt((R + dr)**2 - (var[0] - yo)**2) + xo
# f2: *y = sqrt((R + *dR)^2 - (*x - xo)^2) + yo
def landmark_r_constrant_2(var, R, xo, yo, dr):
    return u_sqrt((R + dr)**2 - (var[0] - xo)**2) + yo

# sqrt((R + *dR)^2 - (*y - yo)^2) >= 0
# sqrt((R + *dR)^2 - (*x - xo)^2) >= 0
def landmark_r_correctness_1(var, R, xyo, dr):
    res = (R + dr)**2 - (var[0] - xyo)**2
    return not (res < 0)
    
'''
Single landmark angle functions
G: *Y = atan[ (yo - *y) / (xo - *x) ] - a - *da
'''
# g1: G
def landmark_a_constrant_1(var, xo, yo, a, da):
    return u_atan2((yo - var[1]),(xo - var[0])) - da - a
# g2: *y = tan(*Y + *da + a) * (*x - xo) + yo        
def landmark_a_constrant_2(var, xo, yo, a, da):
    return yo - u_tan(var[1] + da + a) * (xo - var[0])
# g3: *x = (*y - yo) / tan(*Y + *da + a) + xo
def landmark_a_constrant_3(var, xo, yo, a, da):
    return xo - (yo - var[0]) / u_tan(var[1] + da + a) 

'''
Two bounded landmark angle functions
F: Sin Theorem
    A/sin(alpha) = B/sin(*beta) = C/sin(*gama) = 2R
    A = + sqrt[ (xo1 - xo2)^2 + (yo1 - yo2)^2]
    alpha = (a1 - a2) + 2* *da
    B = sqrt[(xo1 - *x)^2 + (yo1 - *y)^2]
    C = sqrt[(xo2 - *x)^2 + (yo2 - *y)^2]
    alpha + *gama + *beta = pi        
'''                    
def bounded_landmarks_a_alpha(a1, a2, da):
    return u_norm_angle(float(np.abs(llc.substract_angles(a1, a2))) + 2*da)
    
def bounded_landmarks_a_R(xo1, yo1, xo2, yo2, alpha):
    a = float(np.sqrt((xo1 - xo2)**2 + (yo1 - yo2)**2))
    return a / u_sin(alpha) /2

# h1: *X = sqrt[ {2R sin(*beta)}^2 - (Yo1 - *Y)^2 ] + Xo1        
# h2: *Y = sqrt[ {2R sin(*beta)}^2 - (Xo1 - *X)^2 ] + Yo1
# h3: *X = sqrt[ {2R sin(*gama)}^2 - (Yo2 - *Y)^2 ] + Xo2
# h4: *Y = sqrt[ {2R sin(*gama)}^2 - (Xo2 - *X)^2 ] + Yo2
def bounded_landmarks_constrants_13(var, R, xo, yo):
    return u_sqrt( (2*R * u_sin(var[1]) )**2 - (yo - var[0])**2 ) + xo

def bounded_landmarks_constrants_24(var, R, xo, yo):    
    return bounded_landmarks_constrants_13(var, R, yo, xo)

# h5: *beta = pi - *gama - alpha
# h6: *gama = pi - *beta - alpha
def bounded_landmarks_constrants_56(var, alpha):
    return np.pi - var[0] - alpha

# h7: *beta = arcsin[ b / 2R ]
# h8: *gama = arcsin[ c / 2R ]
def bounded_landmarks_constrants_78(var, R, xo, yo):
    bc = u_sqrt((xo - var[0])**2 + (yo - var[1])**2, False)
    return u_arcsin(bc / (2*R))

# Correctness 1
# under arcsin variable of bounded_landmarks_constrants_78 must not lay outside [-1, 1]
def bounded_landmarks_correctness_1(var, R, xo, yo):    
    bc = u_sqrt((xo - var[0])**2 + (yo - var[1])**2, False)    
    res = bc / (2*R)    
    return not (res < -1 or res > 1)

class SDL2D(llc.LandmarkLocalization):
    
    def __init__(self, params = {}, ll_method):
        
        super(SDL2D, self).__init__()
        
        if 'var_acc' not in params:
            params['var_acc'] = 3
        if 'stop_acc' not in params:
            params['stop_acc'] = 0.01
        if 'verbose' not in params:
            params['verbose'] = True
        if 'max_steps' not in params:
            params['max_steps'] = 100
        if 'max_mc_rolls' not in params:
            params['max_mc_rolls'] = 64
        if 'use_correctness_check' not in params:
            params['use_correctness_check'] = True
            
        self.params = params
        
        # probabilistic method works in pair with SDL
        self.ll_method == ll_method
        # SDM
        self.model = SDM(var_acc = self.params['var_acc'],
                         stop_acc = self.params['stop_acc'],
                         verbose = self.params['verbose'],
                         max_steps = self.params['max_steps'],
                         max_mc_rolls = self.params['max_mc_rolls'],
                         use_correctness_check = self.params['use_correctness_check'])
        
        self.x_max = sd_mi([sd_var(self.params['dims']['x']['min']), sd_var(self.params['dims']['x']['max'])])
        self.y_max = sd_mi([sd_var(self.params['dims']['y']['min']), sd_var(self.params['dims']['y']['max'])])
        self.Y_max = sd_mi([sd_var(self.params['dims']['Y']['min']), sd_var(self.params['dims']['Y']['max'])])
        
        self.model.register_variable('x', self.x_max)
        self.model.register_variable('y', self.y_max)
        self.model.register_variable('Y', self.Y_max, 'ANGLE')
        
    def motion_update(self, motion_params):
        # TODO super check params
        #pass
    
    def landmarks_update(self, landmarks_params ):
        #TODO super check params
        pass
    
    def get_pose(self):        
        
        #self.ll_method.params['dims']['x'] = ...
        
        self.ll_method.motion_update()
        
        return self.ll_method.get_pose()                
    
    def calc_cov(self, pose):
        self.ll_method.calc_cov()
        
    def get_cov(self):
        self.ll_method.get_cov()
        
        
if __name__ == '__main__': 
    pass
