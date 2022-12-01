#!/usr/bin/env python
# coding: utf-8

'''
2D Adaptive Monte-Carlo Localization for landmark localization task
'''
import landmark_localization.landmark_localization_core as llc
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm, circmean

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
        multi_interval_constrans:
            x: [[low1, high1], [low2, high2], ...]
            y: ...
            Y: ...
    '''
    def __init__(self, params = {}):
        super(AMCL2D, self).__init__()
        if not 'NP' in params:
            params['NP'] = 100
        if not 'NPmax' in params:
            params['NPmax'] = 100        
        if params['NPmax'] < params['NP']:
            print("amcl: NPmax cannot be less than NP!")
            params['NPmax'] = params['NP']
        if not 'calc_type' in params or (params['calc_type'] != "ADDITION" and params['calc_type'] != "MULTIPLICATION"):
            params['calc_type'] = "ADDITION"                    
        if not 'alpha' in params:
            params['alpha'] = [1,1,1,1,1,1]
        if not 'alpha_slow' in params:
            params['alpha_slow'] = 0.0001
        if not 'alpha_fast' in params:
            params['alpha_fast'] = 0.1            
        
        self.was_landmark_update = False
        
        self.params = params                    
        self.alpha_mat = np.array(params['alpha']).reshape(3,2).T                
        self.init_pf()
        
    def init_pf(self):
        size = (self.params['NP'],1)
        #self.P = np.hstack(self.get_random_pose(size))
        self.P = self.get_random_pose(size)
        #print(self.P.shape)
        self.init_weight()
        self.w_avg = 0
        self.w_slow = 0
        self.w_fast = 0
        
    def get_random_pose(self, size):
        if 'multi_interval_constrans' in self.params:
            if type(size) is int:
                s = size
            if type(size) is tuple:
                s = size[0]
            
            res= np.vstack((self.get_random_value_from_multi_interval(self.params['multi_interval_constrans']['x'], s),
                    self.get_random_value_from_multi_interval(self.params['multi_interval_constrans']['y'], s),
                    self.get_random_value_from_multi_interval(self.params['multi_interval_constrans']['Y'], s))).T
            #print('mic', res.shape)
            if type(size) is int: # NOTE: kostylish
                return res[0,:]
            return res#[:,0,:]
        else:
            res =  np.hstack((np.random.uniform(self.params['dims']['x']['min'], self.params['dims']['x']['max'],size),np.random.uniform(self.params['dims']['y']['min'], self.params['dims']['y']['max'],size),np.random.uniform(self.params['dims']['Y']['min'], self.params['dims']['Y']['max'],size)))
            #print('dims', res.shape)
            return res
        
    '''
    multi_interval (list of list): [[low1, high1], [low2, high2], ...]
    !!! high_i < low_i+1
    '''
    def get_random_value_from_multi_interval(self, multi_interval, size = 1):
        lists_i = [i for i in range(len(multi_interval))]
        lists_w = np.array([l[1] - l[0] for l in multi_interval])
        lists_w = lists_w / np.sum(lists_w)
        if size == 1:
            i = np.random.choice(lists_i, p = lists_w)
            v = np.random.uniform(multi_interval[i][0], multi_interval[i][1])
            return v
        else:
            output = []
            for _ in range(size):
                i = np.random.choice(lists_i, p = lists_w)
                v = np.random.uniform(multi_interval[i][0], multi_interval[i][1])
                output.append(v)
            return output              
        
    def bel_multi_interval(self, value, multi_interval):
        for interval in multi_interval:
            if value <= interval[1] and value >= interval[0]:
                return True
        return False        
        
    def init_weight(self):
        if self.params['calc_type'] == 'ADDITION':            
            self.W = np.zeros(self.P.shape[0])
        else:
            self.W = np.ones(self.P.shape[0])            
            
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
        self.P[:,2] = llc.norm_angle(self.P[:,2])
                
        self.check_borders()        
        
    def check_borders(self):
        if 'multi_interval_constrans' in self.params:
            for j in range(self.P.shape[0]):
                for i, ax in enumerate(['x', 'y', 'Y']):
                    if not self.bel_multi_interval(self.P[j,i], self.params['multi_interval_constrans'][ax]):
                        self.P[j,i] = self.get_random_value_from_multi_interval(self.params['multi_interval_constrans'][ax])
                        self.W[j] = 1./self.W.shape[0]#0.0                        
        else:
            out_indices = []
            for i, ax in enumerate(['x', 'y', 'Y']):                
                add = np.asarray( np.logical_or(self.P[:,i] < self.params['dims'][ax]['min'], self.P[:,i] > self.params['dims'][ax]['max'])).nonzero()                
                out_indices += list(add[0])
                        
            out_indices = list(set(out_indices)) #TODO: change on np.unique sometime
            for i in range(3):
                self.P[out_indices,i] = np.random.uniform(self.params['dims'][ax]['min'],self.params['dims'][ax]['max'], len(out_indices) )            
            self.W[out_indices] = 1./self.W.shape[0]                                
                
                
        
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
                
        if len(rl) != 0 or len(al) != 0:        
            self.was_landmark_update = True
            
        if len(rl) != 0:                        
            rl = np.array(rl, dtype = np.float64)
            mP = np.tile(self.P, [rl.shape[0], 1])
            mrl = np.repeat(rl, self.P.shape[0], axis = 0)
            dx = mrl[:,0] - mP[:,0]
            dy = mrl[:,1] - mP[:,1]                 
            dr = np.hypot(dx, dy) - mrl[:,2]
            w_all = norm.pdf(dr, scale = mrl[:,3])            
            w = w_all.reshape((rl.shape[0], self.P.shape[0]))            
            
            if self.params['calc_type'] == 'ADDITION':
                w = np.sum(w, axis = 0)
                self.W += w
            else:
                w = np.prod(w, axis = 0)
                self.W *= w                            
            
        if len(al) != 0:
            al = np.array(al, dtype = np.float64)        
            mP = np.tile(self.P, [al.shape[0], 1])
            mal = np.repeat(al, self.P.shape[0], axis = 0)
            dx = mal[:,0] - mP[:,0]
            dy = mal[:,1] - mP[:,1]
            a_glob = np.arctan2(-dy, -dx)
            a = llc.substract_angles(a_glob, mP[:,2])
            da = llc.substract_angles(a, mal[:,2])            
            w_all = norm.pdf(da, scale = mal[:,3])            
            w = w_all.reshape((al.shape[0], self.P.shape[0]))            
            
            if self.params['calc_type'] == 'ADDITION':
                w = np.sum(w, axis = 0)
                self.W += w
            else:
                w = np.prod(w, axis = 0)
                self.W *= w                                       

    def get_pose(self):            
        # TODO what if sum(W) close to zero?
        if np.sum(self.W) == 0:
            self.init_pf()
            #return None
            #return [None, None, None]
        
        Wnorm = self.W / np.sum(self.W)
        robot_pose = np.dot(Wnorm, self.P)              
        robot_pose[2] = mean_angles(self.P[:,2].tolist(), self.W.tolist())

        self.cov = self.calc_cov(robot_pose)
        if self.was_landmark_update:            
            self.resampling()
        self.was_landmark_update = False
        return robot_pose            
    
    def calc_cov(self, pose):
        dX = np.zeros(self.P.shape)
        dX[:, :2] = self.P[:,:2] - pose[:2]
        dX[:,2] = llc.substract_angles(self.P[:,2], pose[2])        
        return np.dot(dX * np.expand_dims(self.W, axis=1), dX.T)        

    def resampling(self):
        self.W = np.nan_to_num(self.W) # np.random.choice failing when W contaons NaN
        
        self.w_avg = np.mean(self.W)
        # get NP particles from previous
        N = self.W.shape[0]
        sumW = np.sum(self.W)
        if sumW == 0:
            #self.W = 1
            self.init_weight()
            return 
        
        self.W /= sumW
        #print(self.W)
        
        indexes = np.random.choice(N, size = self.params['NP'], p = self.W)        
        self.P = self.P[indexes,:]
        self.W = self.W[indexes]
        
        # add extra particles 
        Padd = []     
        Wadd = []
        self.w_slow += self.params['alpha_slow'] * (self.w_avg - self.w_slow)
        self.w_fast += self.params['alpha_fast'] * (self.w_avg - self.w_fast)                                
        p = max(0.0, 1.0 - self.w_fast/self.w_slow)        
        for _ in range(self.params['NPmax'] - self.params['NP']):
            if np.random.uniform(0,1) <= p:
                Padd.append(self.get_random_pose(1))
                Wadd.append(0.0)
        if len(Padd) > 0:
            #print(Padd)
            Padd = np.array(Padd)            
            #print(self.P.shape, Padd.shape, 'multi_interval_constrans' in self.params)
            self.P = np.vstack((self.P, Padd))
            self.W = np.hstack((self.W, Wadd))
                              
        #print("resampling w_fast/w_slow = {}, p={}, N={}".format(self.w_fast/self.w_slow, p, len(self.W)))            
        
    def plot(self, lenght = 0.2, color = 'blue'):
        for p in range(self.P.shape[0]):
            plt.arrow(self.P[p,0], self.P[p,1], np.cos(self.P[p,2])*lenght, np.sin(self.P[p,2])*lenght, color = color, shape = 'full', head_width=0.1, alpha = 0.5)
                           
def mean_angles(angles, weights):
    x = y = 0.
    for angle, weight in zip(angles, weights):
        x += np.cos(angle) * weight
        y += np.sin(angle) * weight
    mean = np.arctan2(y, x)
    return mean
        
if __name__ == '__main__': 
    pass
