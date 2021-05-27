#!/usr/bin/env python
# coding: utf-8

'''
2D Adaptive Monte-Carlo Localization for landmark localization task
'''
import landmark_localization.landmark_localization_core as llc
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

class AMCL2D(llc.LandmarkLocalization):
    
    '''
    params dictionary:
        dims:
            x: min, max
            y: min, max
            Y: min, max
        NP(particles amount): 100 (def)
        calc_type: "ADDITION" (def) or "MULTIPLICATION"
        alpha: [1,1,1,1,1,1]
    '''
    def __init__(self, params = {}):
        super(AMCL2D, self).__init__()
        if not 'NP' in params:
            params['NP'] = 100
        if not 'calc_type' in params or (params['calc_type'] != "ADDITION" and params['calc_type'] != "MULTIPLICATION"):
            params['calc_type'] = "ADDITION"                    
        if not 'alpha' in params:
            params['alpha'] = [1,1,1,1,1,1]
        
        self.params = params                    
        self.alpha_mat = np.array(params['alpha']).reshape(3,2).T
        self.init_pf()
        
    def init_pf(self):
        size = (self.params['NP'],1)
        self.P = np.hstack((np.random.uniform(self.params['dims']['x']['min'], self.params['dims']['x']['max'],size),
                            np.random.uniform(self.params['dims']['y']['min'], self.params['dims']['y']['max'],size),
                            np.random.uniform(self.params['dims']['Y']['min'], self.params['dims']['Y']['max'],size)))
        self.init_weight()
        
    def init_weight(self):
        if self.params['calc_type'] == 'ADDITION':
            self.W = np.zeros(self.params['NP'])
        else:
            self.W = np.ones(self.params['NP'])
            
    def motion_update(self, motion_params):
        U = np.array((motion_params['vx'],motion_params['wY'])).reshape(1,2)
        b = np.dot(np.absolute(U), self.alpha_mat)
        U_ = np.copy(U)
        U_.resize((1,3))
        b_ = np.maximum(b, 0.0)
        
        U_1 = np.random.normal(0, b_[0,0], self.P.shape[0])
        U_2 = np.random.normal(0, b_[0,1], self.P.shape[0])
        U_3 = np.random.normal(0, b_[0,2], self.P.shape[0])
        U__ = np.vstack((U_1, U_2, U_3))
        U_ = U_ + U__.T
        
        v2w = U_[:,0] / U_[:,1]
                
        self.P[:,0] = self.P[:,0] + v2w * (-np.sin(self.P[:,2]) + np.sin(self.P[:,2] + U_[:,1] * motion_params['dt']))
        self.P[:,1] = self.P[:,1] + v2w * (np.cos(self.P[:,2]) - np.cos(self.P[:,2] + U_[:,1] * motion_params['dt']))
        self.P[:,2] = self.P[:,2] + (U_[:,1] + U_[:,2]) * motion_params['dt']
        
        self.init_weight()
        
    def landmarks_update(self, landmarks_params ):
        #TODO super check params
        
        # NOTE, don't forget about difference between cam and base link frames
        rl = []
        al = []
        for landmark_param in landmarks_params:
            if 'r' in landmark_param:
               rl.append([landmark_param['x'], landmark_param['y'], landmark_param['r'], landmark_param['sr']]) 
            if 'a' in landmark_param:
                al.append([landmark_param['x'], landmark_param['y'], landmark_param['a'], landmark_param['sa']])
        if len(rl) != 0:
            rl = np.array(rl)
            mP = np.tile(self.P, [rl.shape[0], 1])
            mrl = np.repeat(rl, self.P.shape[0], axis = 0)
            dx = mrl[:,0] - mP[:,0]
            dy = mrl[:,1] - mP[:,1]
            dr = np.hypot(dx, dy)
            w_all = norm.pdf(dr, scale = mrl[:,3])
            print(w_all)
            w = w_all.reshape((rl.shape[0], self.P.shape[0]))
            
            
            if self.params['calc_type'] == 'ADDITION':
                w = np.sum(w, axis = 0)
                self.W += w
            else:
                w = np.prod(w, axis = 0)
                self.W *= w
            
        if len(al) != 0:
            al = np.array(al)
        
        
    def get_pose(self):
        sumW = np.sum(self.W)
        print(sumW)
        if sumW == 0 or sumW == float('nan'):
            self.init_pf()
            sumW = 1
        self.W /= sumW
        robot_pose = np.dot(self.W, self.P)        
        return robot_pose        
        
    def plot(self, lenght = 0.2, color = 'blue'):
        for p in range(self.params['NP']):
            plt.arrow(self.P[p,0], self.P[p,1], np.cos(self.P[p,2])*lenght, np.sin(self.P[p,2])*lenght, color = color, shape = 'full', head_width=0.1)
                           
        
if __name__ == '__main__': 
    pass
