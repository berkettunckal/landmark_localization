#!/usr/bin/env python
# coding: utf-8

'''
test, where robots stands on one position but sometimes teleports 
'''

import numpy as np
import matplotlib.pyplot as plt
from landmark_localization.ll_hf2d import HF2D
from landmark_localization.ll_sdl2d import SDL2D
from landmark_localization.ll_amcl2d import AMCL2D
from landmark_localization.landmark_localization_core import substract_angles, plot_cov
import copy

from landmark_localization.test_utils import *

if __name__ == '__main__':        
    
    test_params = {}
    
    HIST = 1
    AMCL = 1
    SDL_HF = 1
    SDL_AMCL = 1
    
    test_params['sim'] = {}
    test_params['sim']['dt'] = 3
    test_params['sim']['measure_freq'] = 0
    test_params['sim']['steps'] = 1000
    test_params['sim']['step'] = 0
    
    test_params['field'] = {}
    test_params['field']['N_landmarks'] = 20
    test_params['field']['x_max'] = 10
    test_params['field']['y_max'] = 12
    
    test_params['robot'] = {}
    test_params['robot']['x'] = np.random.uniform(-test_params['field']['x_max'], test_params['field']['x_max'])
    test_params['robot']['y'] = np.random.uniform(-test_params['field']['y_max'], test_params['field']['y_max'])
    test_params['robot']['Y'] = np.random.uniform(-np.pi, np.pi)
    
    test_params['robot']['v'] = 0.0001
    test_params['robot']['w'] = 0.0001
    test_params['robot']['sv'] = 0.001
    test_params['robot']['sw'] = 0.001
    
    test_params['sensor'] = {}
    test_params['sensor']['max_r'] = 10
    test_params['sensor']['max_a'] = np.pi
    test_params['sensor']['sr'] = 0.1
    test_params['sensor']['sa'] = 0.01
    
    landmarks = generate_landmarks(test_params)        
    #TODO: read/write params from file    
    
    real_history = []
    '''
    localization methods
    '''    
    hf_params = {}
    hf_params['dims'] = {}
    hf_params['dims']['x'] = {}
    hf_params['dims']['x']['min'] = -test_params['field']['x_max']
    hf_params['dims']['x']['max'] = test_params['field']['x_max']
    hf_params['dims']['x']['d_res'] = 0.2
    hf_params['dims']['y'] = {}
    hf_params['dims']['y']['min'] = -test_params['field']['y_max']
    hf_params['dims']['y']['max'] = test_params['field']['y_max']
    hf_params['dims']['y']['d_res'] = 0.2
    hf_params['dims']['Y'] = {}
    hf_params['dims']['Y']['min'] = -np.pi
    hf_params['dims']['Y']['max'] = np.pi
    hf_params['dims']['Y']['d_res'] = 0.15
    hf_params['calc_type'] = "ADDITION"
    #hf_params['calc_type'] = "MULTIPLICATION" # NOTE: not works well
    hf_params['yaw_discount'] = 0.1
    hf_params['prev_step_weight'] = 0.5
    hf_params['motion_update_type'] = 'PREV_COV'
    hf_params['pose_calc_type'] = 'MAX'
    hf = HF2D(hf_params)
    hf_history = []
    
    amcl_params = {}
    amcl_params['NP'] = 1000
    amcl_params['NPmax'] = 2000
    amcl_params['alpha_slow'] = 0.05
    amcl_params['alpha_fast'] = 0.1
    amcl_params['dims'] = {}
    amcl_params['dims']['x'] = {}
    amcl_params['dims']['x']['min'] = -test_params['field']['x_max']
    amcl_params['dims']['x']['max'] = test_params['field']['x_max']
    amcl_params['dims']['y'] = {}
    amcl_params['dims']['y']['min'] = -test_params['field']['y_max']
    amcl_params['dims']['y']['max'] = test_params['field']['y_max']
    amcl_params['dims']['Y'] = {}
    amcl_params['dims']['Y']['min'] = -np.pi
    amcl_params['dims']['Y']['max'] = np.pi
    amcl_params['calc_type'] = "MULTIPLICATION"
    amcl = AMCL2D(amcl_params)
    amcl_history = []
    
    sdl_hf_params = {}
    sdl_hf_params['verbose'] = False
    sdl_hf_params['dims'] = {}
    sdl_hf_params['dims']['x'] = {}
    sdl_hf_params['dims']['x']['min'] = -test_params['field']['x_max']
    sdl_hf_params['dims']['x']['max'] = test_params['field']['x_max']
    sdl_hf_params['dims']['y'] = {}
    sdl_hf_params['dims']['y']['min'] = -test_params['field']['y_max']
    sdl_hf_params['dims']['y']['max'] = test_params['field']['y_max']
    sdl_hf_params['dims']['Y'] = {}
    sdl_hf_params['dims']['Y']['min'] = -np.pi
    sdl_hf_params['dims']['Y']['max'] = np.pi
    sdl_hf_params['inner_method'] = 'hf'
    sdl_hf_params['inner_method_params'] = copy.deepcopy(hf_params)    
    #sdl_hf_params['inner_method_params']['pose_calc_type'] = 'SUM'
    sdl_hf = SDL2D(sdl_hf_params)
    sdl_hf_history = []
    
    sdl_amcl_params = {}
    sdl_amcl_params['verbose'] = False
    sdl_amcl_params['dims'] = {}
    sdl_amcl_params['dims']['x'] = {}
    sdl_amcl_params['dims']['x']['min'] = -test_params['field']['x_max']
    sdl_amcl_params['dims']['x']['max'] = test_params['field']['x_max']
    sdl_amcl_params['dims']['y'] = {}
    sdl_amcl_params['dims']['y']['min'] = -test_params['field']['y_max']
    sdl_amcl_params['dims']['y']['max'] = test_params['field']['y_max']
    sdl_amcl_params['dims']['Y'] = {}
    sdl_amcl_params['dims']['Y']['min'] = -np.pi
    sdl_amcl_params['dims']['Y']['max'] = np.pi    
    sdl_amcl_params['inner_method'] = 'amcl'
    sdl_amcl_params['inner_method_params'] = copy.deepcopy(amcl_params)
    sdl_amcl = SDL2D(sdl_amcl_params)
    sdl_amcl_history = []
    
    measure_freq_cnt = 0    
    field_figure = plt.figure('field')           
    
    teleport_chance = 0.2
    
    while test_params['sim']['step'] < test_params['sim']['steps']:        
        plt.figure('field')
        test_params['sim']['step'] += 1
        
        # MOTION
        motion_params = {}
        motion_params['dt'] = test_params['sim']['dt']    
        motion_params['wY'] = test_params['robot']['w'] 
        motion_params['swY'] = test_params['robot']['sw']    
        motion_params['vx'] = test_params['robot']['v'] 
        motion_params['svx'] = test_params['robot']['sv']
        
        if np.random.uniform(0,1) <= teleport_chance:
            test_params['robot']['x'] = np.random.uniform(-test_params['field']['x_max'], test_params['field']['x_max'])
            test_params['robot']['y'] = np.random.uniform(-test_params['field']['y_max'], test_params['field']['y_max'])
            test_params['robot']['Y'] = np.random.uniform(-np.pi, np.pi)
            
        
        real_history.append([test_params['robot']['x'], test_params['robot']['y'], test_params['robot']['Y']])
        if HIST:
            hf.motion_update(motion_params)
        if AMCL:
            amcl.motion_update(motion_params)
        if SDL_HF:
            sdl_hf.motion_update(motion_params)
        if SDL_AMCL:
            sdl_amcl.motion_update(motion_params)
                
        # MEASURE
        landmarks_params = []
        if measure_freq_cnt == test_params['sim']['measure_freq'] or test_params['sim']['step'] == 1:            
            landmarks_params = do_measure(test_params, landmarks)
            if HIST:
                hf.landmarks_update(landmarks_params)
            if AMCL:
                amcl.landmarks_update(landmarks_params)
            if SDL_HF:
                sdl_hf.landmarks_update(landmarks_params)
            if SDL_AMCL:
                sdl_amcl.landmarks_update(landmarks_params)
            measure_freq_cnt = 0
        else:
            measure_freq_cnt += 1
                                
        # GET POSES
        if HIST:
            hf_pose = hf.get_pose()
            hf_history.append(hf_pose)
        if AMCL:
            amcl_pose = amcl.get_pose()        
            amcl_history.append(amcl_pose)
        if SDL_HF:
            sdl_hf_pose = sdl_hf.get_pose()
            sdl_hf_history.append(sdl_hf_pose)
        if SDL_AMCL:
            sdl_amcl_pose = sdl_amcl.get_pose()
            sdl_amcl_history.append(sdl_amcl_pose)
                
        # PLOT STUFF
        plot_exp_base(field_figure, test_params, landmarks_params, landmarks)        
        plt.plot(np.array(real_history)[:,0], np.array(real_history)[:,1], '-r')
        
        if HIST:
            hf.plot()        
            plot_robot_pose(hf_pose[0], hf_pose[1], hf_pose[2], "green", "hf")                
            plt.plot(np.array(hf_history)[:,0], np.array(hf_history)[:,1], '-g')        
            plot_cov(plt.gca(), hf_pose, hf.get_cov(), color = 'g')
        
        if AMCL:
            amcl.plot()
            plot_robot_pose(amcl_pose[0], amcl_pose[1], amcl_pose[2], "blue", "amcl")                
            plt.plot(np.array(amcl_history)[:,0], np.array(amcl_history)[:,1], '-b')
            plot_cov(plt.gca(), amcl_pose, amcl.get_cov(), color = 'b')
            
        if SDL_HF:
            sdl_hf.plot()
            plot_robot_pose(sdl_hf_pose[0], sdl_hf_pose[1], sdl_hf_pose[2], "magenta", "sdl+hf")                
            plt.plot(np.array(sdl_hf_history)[:,0], np.array(sdl_hf_history)[:,1], '-m')        
            plot_cov(plt.gca(), sdl_hf_pose, sdl_hf.get_cov(), color = 'm')
            
        if SDL_AMCL:
            sdl_amcl.plot()
            plot_robot_pose(sdl_amcl_pose[0], sdl_amcl_pose[1], sdl_amcl_pose[2], "cyan", "sdl+amcl")
            plt.plot(np.array(sdl_amcl_history)[:,0], np.array(sdl_amcl_history)[:,1], '-c')
            #plot_cov(plt.gca(), sdl_amcl_pose, sdl_amcl.get_cov(), color = 'c')
        
        plt.legend()
        plt.grid()
        plt.pause(0.01)
        
        plt.figure('boxplot')
        plt.cla()
        data = []
        labels = []
        colors = []
        if HIST:
            data.append(get_pose_errors(hf_history, real_history))
            labels.append('hf')
            colors.append('green')
        if AMCL:
            data.append(get_pose_errors(amcl_history, real_history))
            labels.append('amcl')
            colors.append('blue')
        if SDL_HF:
            data.append(get_pose_errors(sdl_hf_history, real_history))
            labels.append('sdl+hf')
            colors.append('magenta')
        if SDL_AMCL:
            data.append(get_pose_errors(sdl_amcl_history, real_history))
            labels.append('sdl+amcl')        
            colors.append('cyan')

        moving_boxplot(data, labels, colors)
        
        plt.pause(0.01)
            
