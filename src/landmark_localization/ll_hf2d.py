#!/usr/bin/env python
# coding: utf-8

'''
2D Histogram Filter for landmark localization task
'''
import landmark_localization.landmark_localization_core as llc
import numpy as np
from scipy.stats import norm
import matplotlib.pyplot as plt

class HF2D(llc.LandmarkLocalization):
    
    '''
    params dictinary:
        dims:
            x: d_res, min, max
            y: d_res, min, max                        
            Y: d_res, min, max  
        calc_type: "ADDITION" or "MULTIPLICATION"
    '''
    def __init__(self, params = {}):
        super(HF2D, self).__init__()        
        
        if not 'calc_type' in params or (params['calc_type'] != "ADDITION" and params['calc_type'] != "MULTIPLICATION"):
            params['calc_type'] = "ADDITION"                    
        # TODO check other params
        self.params = params
        
        self.grid_size = tuple()
        for dim in self.params['dims'].values():        
            self.count_dim(dim)
            self.grid_size += (dim['size'],)
            
        self.m_grid = self.reset_grid() # main grid throught steps
        self.s_grid = self.reset_grid() # new step grid
        
        self.x_ls = self.get_linspace(self.params['dims']['x'])
        self.y_ls = self.get_linspace(self.params['dims']['y'])
        self.Y_ls = self.get_linspace(self.params['dims']['Y'])
        
        self.xx_mg, self.yy_mg = np.meshgrid(self.x_ls, self.y_ls)
        
        self.plot_mx, self.plot_my = np.mgrid[slice(self.params['dims']['x']['min'],
                                self.params['dims']['x']['min'] + self.params['dims']['x']['res'] * (self.params['dims']['x']['size']),
                                self.params['dims']['x']['res']),
                                    slice(self.params['dims']['y']['min'],                                
                                self.params['dims']['y']['min'] + self.params['dims']['y']['res'] * (self.params['dims']['y']['size']),
                                self.params['dims']['y']['res'])]
        
    def get_linspace(self, dim):
        return np.linspace(dim['min'], dim['max'], dim['size'])        
        
    def reset_grid(self):
        if self.params['calc_type'] == "ADDITION":            
            grid = np.zeros(self.grid_size)
        else:
            grid = np.ones(self.grid_size)
        return grid
                    
    def count_dim(self, dim):
        dim_len = dim['max'] - dim['min']
        if dim_len > 0:
            dim['size'] = int(np.ceil(dim_len / dim['d_res']))
            dim['res'] = dim_len / dim['size']
        else:
            dim['min_len'] -= dim['d_res']/2
            dim['max_len'] += dim['d_res']/2
            dim['size'] = 1
            dim['res'] = dim['d_res']
    
    
    def motion_update(self, motion_params):
        # TODO super check params
        pass            
    
    # NOTE that function could be more vectorized
    def landmarks_update(self, landmarks_params ):
        #TODO super check params
                
        for landmark_param in landmarks_params:            
            x = self.xx_mg - landmark_param['x']
            y = self.yy_mg - landmark_param['y']
                                    
            if 'r' in landmark_param:
                r = np.hypot(x, y).T
                dr = landmark_param['r'] - r
                
                pxy = norm.pdf(dr, scale = landmark_param['sr'])
                
                if self.params['calc_type'] == "ADDITION":                            
                    self.m_grid += np.expand_dims(pxy, 2)
                else:
                    self.m_grid *= np.expand_dims(pxy, 2)
            
            if 'a' in landmark_param:
                a_glob = np.arctan2(y, x).T                                                                
                a_glob_full_mesh = np.tile(a_glob, (self.params['dims']['Y']['size'],1,1))
                a_glob_full_mesh = np.moveaxis(a_glob_full_mesh, 0, 2)
                
                a = llc.substract_angles(a_glob_full_mesh, np.expand_dims(self.Y_ls, (0,1)))
                
                da = llc.substract_angles(a, landmark_param['a'])
                
                pY = norm.pdf(da, scale = landmark_param['sa'])
                
                if self.params['calc_type'] == "ADDITION":                            
                    self.m_grid += pY
                else:
                    self.m_grid *= pY                
        
    def get_pose(self):        
        ipose = np.unravel_index(np.argmax(self.m_grid), self.m_grid.shape)        
        pose = []
        for i, p in enumerate(ipose):
            pose.append(list(self.params['dims'].values())[i]['min'] + list(self.params['dims'].values())[i]['res'] * p)                
        self.m_grid = self.reset_grid()                
        return pose
    
    def plot(self):
        plt.pcolor(self.plot_mx, self.plot_my, np.sum(self.m_grid, axis=2), cmap=plt.cm.get_cmap("Reds"), vmin = 0, edgecolors = 'k', alpha = 0.25)
        
if __name__ == '__main__': 
    pass
