#!/usr/bin/env python
# coding: utf-8

'''
2D Sub Defenite Localization plugin
'''

import landmark_localization.landmark_localization_core as llc
import numpy as np
from landmark_localization.sub_def_variable import sd_var
from landmark_localization.sub_def_multi_interval import sd_mi
from landmark_localization.sub_def_model import SDM, u_sqrt, u_atan2, u_tan, u_sin, u_cos, u_norm_angle, u_arcsin, get_rov_monte_carlo_new
from landmark_localization.ll_hf2d import HF2D
from landmark_localization.ll_amcl2d import AMCL2D
from functools import partial
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import copy    

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
    return u_atan2((var[1] - yo),(var[0] - xo)) - da - a
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
    
    def __init__(self, params = {}):
        
        super(SDL2D, self).__init__()
        
        if not 'var_acc' in params:
            params['var_acc'] = 3
        if not 'stop_acc' in params:
            params['stop_acc'] = 0.01
        if not 'verbose' in params:
            params['verbose'] = True
        if not 'max_steps' in params:
            params['max_steps'] = 100
        if not 'max_mc_rolls' in params:
            params['max_mc_rolls'] = 64
        if not 'use_correctness_check' in params:
            params['use_correctness_check'] = True
        if not 'ignore_v_translate' in params:
            params['ignore_v_translate'] = 0.01
        if not 'ignore_w_translate' in params:
            params['ignore_w_translate'] = 0.01
        
        # probabilistic method works in pair with SDL
        if not 'inner_method' in params or not 'inner_method_params' in params:
            print('SDL error: inner_method or its params are not specified')
            self.ll_method = llc.LandmarkLocalization
        elif params['inner_method'] == 'none':
            print('Warn: passing none as inner methond allows find out only some area where robot is')
        elif params['inner_method'] == 'hf':
            #self.ll_method = HF2D(params['inner_method_params'])
            self.inner_hfs = []
            self.inner_hf_prev_pose = None
            self.inner_hf_cov = None
            self.inner_hf_poses = []
            self.inner_hf_weights = []
        elif params['inner_method'] == 'amcl':
            #self.ll_method = AMCL2D(params['inner_method_params'])
            self.inner_amcl = None
        else:
            print('SDL error: unknown inner_method {}'.format(params['inner_method']))
            self.ll_method = llc.LandmarkLocalization
            
        self.params = params                
        
        # SDM
        self.model = SDM(var_acc = self.params['var_acc'],
                         stop_acc = self.params['stop_acc'],
                         verbose = self.params['verbose'],
                         max_steps = self.params['max_steps'],
                         max_mc_rolls = self.params['max_mc_rolls'],
                         use_correctness_check = self.params['use_correctness_check'])
        
        self.x_max = sd_mi([sd_var(self.params['dims']['x']['min'], self.params['dims']['x']['max'])])
        self.y_max = sd_mi([sd_var(self.params['dims']['y']['min'], self.params['dims']['y']['max'])])
        self.Y_max = sd_mi([sd_var(self.params['dims']['Y']['min'], self.params['dims']['Y']['max'])])
        
        self.model.register_variable('x', self.x_max)
        self.model.register_variable('y', self.y_max)
        self.model.register_variable('Y', self.Y_max, 'ANGLE')
        
        self.current_motion_params = None
        self.current_landmarks_params = []

    def sigma_to_sd_mi(self, sigma, mul = 4):
        return sd_mi([sd_var(-sigma * mul, sigma * mul)])              
        
    def motion_update(self, motion_params):
        # TODO super check params                        
        
        self.current_motion_params = copy.deepcopy(motion_params)
        
        if self.current_motion_params['vx'] > self.params['ignore_v_translate'] or self.current_motion_params['wY'] > self.params['ignore_w_translate']:
                    
            dY = motion_params['dt'] * (motion_params['wY'] + self.sigma_to_sd_mi(motion_params['swY']))
            dR = motion_params['dt'] * (motion_params['vx'] + self.sigma_to_sd_mi(motion_params['svx']))
            
            self.model.variables['Y']['VALUE'] = self.Y_max.assign((self.model.variables['Y']['VALUE'] + dY).norm_angle())
            
            def dx(input_vars):
                return input_vars[0] * u_cos(input_vars[1])
            
            def dy(input_vars):
                return input_vars[0] * u_sin(input_vars[1])
            
            self.model.variables['x']['VALUE'] = self.model.variables['x']['VALUE'] + get_rov_monte_carlo_new(dx, [dR, self.model.variables['Y']['VALUE']])
            self.model.variables['x']['VALUE'] =  self.x_max.assign(self.model.variables['x']['VALUE'])
            
            self.model.variables['y']['VALUE'] = self.model.variables['y']['VALUE']+ get_rov_monte_carlo_new(dy, [dR, self.model.variables['Y']['VALUE']])
            self.model.variables['y']['VALUE'] = self.y_max.assign(self.model.variables['y']['VALUE'])                        
            #self.plot("blue")                                           
        
        # clear notmain variables
        to_del = []
        for k, var in self.model.variables.items():
            if k != 'x' and k!= 'y' and k!='Y':
                to_del.append(k)
        for key in to_del: del self.model.variables[key]
        
        # clear previous functions
        self.model.constrants = {}
        self.model.correctnesses = {}
                    
    def landmarks_update(self, landmarks_params ):
        #TODO super check params
        
        self.current_landmarks_params = copy.deepcopy(landmarks_params)
        
        for i, landmark_param in enumerate(landmarks_params):
            if 'r' in landmark_param:
                dr = self.sigma_to_sd_mi(landmark_param['sr'])
                
                self.model.register_constrant("l_r{}.1".format(i),
                                              partial(landmark_r_constrant_1, R = landmark_param['r'], xo = landmark_param['x'], yo = landmark_param['y'], dr = dr),
                                              'x', ['y'],
                                              'ROV_MONTE_CARLO')
                
                self.model.register_constrant("l_r{}.2".format(i),
                                              partial(landmark_r_constrant_2, R = landmark_param['r'], xo = landmark_param['x'], yo = landmark_param['y'], dr = dr),
                                              'y', ['x'],
                                              'ROV_MONTE_CARLO')
                
                if self.params['use_correctness_check']:
                    self.model.register_correctness('l_r{}.1_c'.format(i),
                                                    partial(landmark_r_correctness_1, R = landmark_param['r'], 
                                                            xyo = landmark_param['x'],
                                                            dr = dr),
                                                    ['x'])
                                                    
                    self.model.register_correctness('l_r{}.2_c'.format(i),
                                                    partial(landmark_r_correctness_1, R = landmark_param['r'], 
                                                            xyo = landmark_param['y'],
                                                            dr = dr),
                                                    ['y'])   
            if 'a' in landmark_param:
                da = self.sigma_to_sd_mi(landmark_param['sa'])
                
                self.model.register_constrant('l_a{}.1'.format(i),
                                        partial(landmark_a_constrant_1, xo = landmark_param['x'], 
                                                yo = landmark_param['y'], a = landmark_param['a'], da = da),
                                        'Y', ['x','y'])         
                self.model.register_constrant('l_a{}.2'.format(i),
                                        partial(landmark_a_constrant_2, xo = landmark_param['x'], 
                                                yo = landmark_param['y'], a = landmark_param['a'], da = da),
                                        'y', ['x','Y'], 'ROV_MONTE_CARLO')         
                self.model.register_constrant('l_a{}.3'.format(i),
                                        partial(landmark_a_constrant_3, xo = landmark_param['x'], 
                                                yo = landmark_param['y'], a = landmark_param['a'], da = da),
                                        'x', ['y','Y'], 'ROV_MONTE_CARLO') 
    
    def get_pose(self):                       
        # check correctness 
        # TODO instead of this, get product and remove elements that not correct
        if self.params['use_correctness_check']:
            for name_corr_func in self.model.correctnesses.keys():            
                if not self.model.check_correctness_new(name_corr_func):
                    self.model.variables['x']['VALUE'] = copy.deepcopy(self.x_max)
                    self.model.variables['y']['VALUE'] = copy.deepcopy(self.y_max)
                    self.model.variables['Y']['VALUE'] = copy.deepcopy(self.Y_max)
                    print("\nInput variables was reseted!\n")
                    break
        
        self.model.proc_complex()      
        pose = None
        
        if self.params['inner_method'] == 'hf':
            self.inner_hfs = []
            self.inner_hf_poses = []
            self.inner_hf_weights = []
            
            for x in self.model.variables['x']['VALUE']:
                for y in self.model.variables['y']['VALUE']:
                    for Y in self.model.variables['Y']['VALUE']:
                        hf_params = copy.deepcopy(self.params['inner_method_params'])                        
                        hf_params['dims']['x']['min'] = x.low
                        hf_params['dims']['x']['max'] = x.high
                        hf_params['dims']['y']['min'] = y.low
                        hf_params['dims']['y']['max'] = y.high
                        hf_params['dims']['Y']['min'] = Y.low
                        hf_params['dims']['Y']['max'] = Y.high
                        
                        hf = HF2D(hf_params)
                        hf.prev_pose = self.inner_hf_prev_pose
                        hf.cov = self.inner_hf_cov
                        if not self.current_motion_params is None:
                            hf.motion_update(self.current_motion_params)
                        if not self.current_landmarks_params is None:
                            hf.landmarks_update(self.current_landmarks_params)
                        self.inner_hf_poses.append(hf.get_pose())
                        self.inner_hf_weights.append(hf.get_weight())                                                
                        self.inner_hfs.append(hf)
                        
            best_hf_index = self.inner_hf_weights.index(max(self.inner_hf_weights))
            pose = self.inner_hf_poses[best_hf_index]
            self.inner_hf_prev_pose = pose
            self.inner_hf_cov = self.inner_hfs[best_hf_index].get_cov()
            
        elif self.params['inner_method'] == 'amcl':                    
            # we got real multi interval here
            if len(self.model.variables['Y']) > 1 or len(self.model.variables['x']) > 1 or len(self.model.variables['y']) > 1:
                mi_param = {}
                for ax in ['x', 'y', 'Y']:
                    mi_param[ax] = self.model.variables[ax]['VALUE'].to_list('list')                
                    
                self.params['inner_method_params']['multi_interval_constrans'] = mi_param
            # just one
            else:
                if 'multi_interval_constrans' in self.params['inner_method_params']:
                    del self.params['inner_method_params']['multi_interval_constrans']
                for ax in ['x', 'y', 'Y']:
                    self.params['inner_method_params']['dims'][ax]['min'] = self.model.variables[ax]['VALUE'][0].low
                    self.params['inner_method_params']['dims'][ax]['min'] = self.model.variables[ax]['VALUE'][0].low
                        
            if self.inner_amcl is None:
                self.inner_amcl = AMCL2D(self.params['inner_method_params'])
            else:
                self.inner_amcl.params = self.params['inner_method_params']
            
            if not self.current_motion_params is None:
                self.inner_amcl.motion_update(self.current_motion_params)
            if not self.current_landmarks_params is None:
                self.inner_amcl.landmarks_update(self.current_landmarks_params)
            pose = self.inner_amcl.get_pose()  
                    
        self.current_landmarks_params = []
                    
        return pose
    
    def calc_cov(self, pose):
        if self.params['inner_method'] == 'amcl':
            #self.inner_amcl.calc_cov()
            pass
        else:
            pass
            #self.inner_hf_cov = 0
            #for hf in self.inner_hfs:
                #self.inner_hf_cov += hf.get_cov()
                
    def get_cov(self):
        if self.params['inner_method'] == 'amcl':
            self.inner_amcl.get_cov()
        else:
            return self.inner_hf_cov
        
    def plot(self, color = 'magenta'):
        ax = plt.gca()
        
        for x in self.model.variables['x']['VALUE']:
            cx = (x.high + x.low)/2
            for y in self.model.variables['y']['VALUE']:
                ax.add_patch(mpatches.Rectangle((x.low, y.low), x.high - x.low, y.high - y.low, ec=color, fill = False, ls='--'))
                cy = (y.high + y.low)/2
                for yaw in self.model.variables['Y']['VALUE']:                
                    plt.plot([cx, cx + np.cos(yaw.low)], [cy, cy+np.sin(yaw.low)],color=color,ls="-")
                    plt.plot([cx, cx + np.cos(yaw.high)], [cy, cy+np.sin(yaw.high)],color=color,ls="-")
                    
        if self.params['inner_method'] == 'hf':
            for hf in self.inner_hfs:                
                hf.plot()
        
        if self.params['inner_method'] == 'amcl':
            self.inner_amcl.plot(color = color)
            
        
        
if __name__ == '__main__': 
    pass
