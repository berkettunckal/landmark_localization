#!/usr/bin/env python
# coding: utf-8

'''
simple test, where robot moves through landmarks field with constant linear and angular speed
'''

import numpy as np
import matplotlib.pyplot as plt
from landmark_localization.ll_hf2d import HF2D
from landmark_localization.ll_sdl2d import SDL2D
from landmark_localization.ll_amcl2d import AMCL2D
from landmark_localization.landmark_localization_core import substract_angles, plot_cov
import copy

def generate_landmarks(test_params):
    landmarks = []
    for i in range(test_params['field']['N_landmarks']):
        landmark = {}
        landmark['x'] = np.random.uniform(-test_params['field']['x_max'], test_params['field']['x_max'])
        landmark['y'] = np.random.uniform(-test_params['field']['y_max'], test_params['field']['y_max'])
        landmarks.append(landmark)
    return landmarks  

def do_motion(test_params):
    # real
    test_params['robot']['Y'] += test_params['sim']['dt'] * test_params['robot']['w']
    test_params['robot']['Y'] = (test_params['robot']['Y'] + np.pi)%(2*np.pi) - np.pi
    test_params['robot']['x'] += (test_params['sim']['dt'] * test_params['robot']['v']) * np.cos(test_params['robot']['Y'])
    test_params['robot']['y'] += (test_params['sim']['dt'] * test_params['robot']['v']) * np.sin(test_params['robot']['Y'])
    
    # noisy
    motion_params = {}
    motion_params['dt'] = test_params['sim']['dt']    
    motion_params['wY'] = np.random.normal(test_params['robot']['w'], test_params['robot']['sw'])
    motion_params['swY'] = test_params['robot']['sw']    
    motion_params['vx'] = np.random.normal(test_params['robot']['v'], test_params['robot']['sv'])
    motion_params['svx'] = test_params['robot']['sv']
    
    return motion_params      

def do_measure(test_params, landmarks):
    landmarks_params = []
    for landmark in landmarks:
        dx = test_params['robot']['x'] - landmark['x']
        dy = test_params['robot']['y'] - landmark['y']
        r = np.hypot(dx, dy)
        a_glob = np.arctan2(dy, dx)
        a = substract_angles(a_glob, test_params['robot']['Y'])
        
        if r <= test_params['sensor']['max_r'] and np.abs(a) <= test_params['sensor']['max_a']:
            landmark_param = {}
            landmark_param['x'] = landmark['x']
            landmark_param['y'] = landmark['y']
            landmark_param['r'] = np.random.normal(r, test_params['sensor']['sr'])
            landmark_param['sr'] = test_params['sensor']['sr']
            landmark_param['a'] = np.random.normal(a, test_params['sensor']['sa'])
            landmark_param['sa'] = test_params['sensor']['sa']                    
            landmarks_params.append(landmark_param)
    return landmarks_params        

def plot_robot_pose(x, y, Y, color, label = None):
    plt.plot(x, y, "o", color = color)
    plt.plot([x, x + np.cos(Y)], [y, y + np.sin(Y)], "-", color = color, label = label)
    plt.legend()    

def plot_exp_base(figure, test_params, landmarks_params, landmarks, padding = 1):
    figure.clf()
    plt.xlim(-test_params['field']['x_max'] - padding, test_params['field']['x_max'] + padding)
    plt.ylim(-test_params['field']['y_max'] - padding, test_params['field']['y_max'] + padding)
    ax = plt.gca()    
    ax.set_aspect('equal', 'box')
    plt.title("Sim step {} of {}, dt = {}".format(test_params['sim']['step'], test_params['sim']['steps'], test_params['sim']['dt']))
    
    
    for landmark_param in landmarks_params:
        plt.plot(landmark_param['x'], landmark_param['y'], 'ro')
    
    for landmark in landmarks:
        plt.plot(landmark['x'], landmark['y'], 'k.')
        
    plot_robot_pose(test_params['robot']['x'], test_params['robot']['y'], test_params['robot']['Y'], 'red', 'real_pose')        
    
    
def boxplot( data, ax, labels, color = 'r', rotation  = 0):
    #means = np.mean(data, axis=1)
    bp = ax.boxplot(data, sym='.'+color)
    plt.setp(bp['medians'],color=color, linewidth=3)    
    plt.xticks(range(1,1+len(data)), labels, rotation = rotation)    
    
def get_pose_errors(data, real):
    d = np.array(data)
    r = np.array(real)
    dr = np.hypot(d[:,0] - r[:,0], d[:,1] - r[:,1])
    #print(dr)
    dr = dr[~np.isnan(dr)]
    #print(dr)
    return dr
    

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
    test_params['field']['N_landmarks'] = 10
    test_params['field']['x_max'] = 10
    test_params['field']['y_max'] = 12
    
    test_params['robot'] = {}
    test_params['robot']['x'] = 0
    test_params['robot']['y'] = -9
    test_params['robot']['Y'] = 0
    test_params['robot']['v'] = 0.1
    test_params['robot']['w'] = 0.012
    test_params['robot']['sv'] = 0.01
    test_params['robot']['sw'] = 0.01
    
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
    #plt.pause(2)
    while test_params['sim']['step'] < test_params['sim']['steps']:
        #field_figure.clf() # TODO: remove it
        plt.figure('field')
        test_params['sim']['step'] += 1
        
        # MOTION
        motion_params = do_motion(test_params)
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
        if HIST:
            data.append(get_pose_errors(hf_history, real_history))
            labels.append('hf')
        if AMCL:
            data.append(get_pose_errors(amcl_history, real_history))
            labels.append('amcl')
        if SDL_HF:
            data.append(get_pose_errors(sdl_hf_history, real_history))
            labels.append('sdl+hf')
        if SDL_AMCL:
            data.append(get_pose_errors(sdl_amcl_history, real_history))
            labels.append('sdl+amcl')        

        data_labels = list(zip(data, labels))
        
        #data_labels = [d for d in sorted(data_labels, key=lambda item: np.median(item[0]))]
        data_labels = sorted(data_labels, key=lambda item: np.median(item[0]) )
        #labels = [d for in sorted(data), key=lambda item: np.median(item)]
        
        data_labels = list(zip(*data_labels))
        
        #print(data_labels[0])
        
        boxplot(data_labels[0], plt.gca(), data_labels[1])
        plt.grid()
        
        plt.pause(0.01)
    
    
    
