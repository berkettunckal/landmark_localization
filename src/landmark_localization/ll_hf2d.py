#!/usr/bin/env python
# coding: utf-8

'''
2D Histogram Filter for landmark localization task
'''
import landmark_localization.landmark_localization_core as llc
import numpy as np
from scipy.stats import norm
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter

class HF2D(llc.LandmarkLocalization):
    
    '''
    params dictionary:
        dims:
            x: d_res, min, max
            y: d_res, min, max                        
            Y: d_res, min, max  
        calc_type: "ADDITION" (def) or "MULTIPLICATION"
        yaw_discount: 1 (def)
        prev_step_weight: 0.5 (def)
    '''
    def __init__(self, params = {}):
        super(HF2D, self).__init__()        
        
        if not 'calc_type' in params or (params['calc_type'] != "ADDITION" and params['calc_type'] != "MULTIPLICATION"):
            params['calc_type'] = "ADDITION"                    
        if not 'yaw_discount' in params:
            params['yaw_discount'] = 1
        if not 'prev_step_weight' in params:
            params['prev_step_weight'] = 0.5
        
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
        
        # TODO stash motion between steps without landmark update
        r = motion_params['dt'] * motion_params['vx']
        da = motion_params['dt'] * motion_params['wY']
        a_shift = int(da / self.params['dims']['Y']['res'])                
        
        if( a_shift > 0 ):            
            self.m_grid[:,:,:-a_shift] = self.m_grid[:,:,a_shift:]
        elif a_shift < 0:
            self.m_grid[:,:,a_shift:] = self.m_grid[:,:,:-a_shift]
        # NOTE: idk how to blur yaw axis
        
        for yaw_row in range(self.params['dims']['Y']['size']):            
            a = self.params['dims']['Y']['min'] + self.params['dims']['Y']['res'] * yaw_row
            
            x_shift = int(r * np.cos(a) / self.params['dims']['x']['res'])
            y_shift = int(r * np.sin(a) / self.params['dims']['y']['res'])                                
            
            if( x_shift > 0 ):            
                self.m_grid[:-x_shift,:,yaw_row] = self.m_grid[x_shift:,:,yaw_row]
            elif x_shift < 0:
                self.m_grid[x_shift:,:,yaw_row] = self.m_grid[:-x_shift,:,yaw_row]
            
            if( y_shift > 0 ):            
                self.m_grid[:,:-y_shift,yaw_row] = self.m_grid[:,y_shift:,yaw_row]
            elif y_shift < 0:
                self.m_grid[:,y_shift:,yaw_row] = self.m_grid[:,:-y_shift,yaw_row]
        
            self.m_grid[:,:,yaw_row] = gaussian_filter(self.m_grid[:,:,yaw_row], sigma = motion_params['svx'] * r)
            
        self.s_grid = self.reset_grid()
    
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
                    self.s_grid += np.expand_dims(pxy, 2)
                else:
                    self.s_grid *= np.expand_dims(pxy, 2)
            
            if 'a' in landmark_param:
                a_glob = np.arctan2(y, x).T                                                                
                a_glob_full_mesh = np.tile(a_glob, (self.params['dims']['Y']['size'],1,1))
                a_glob_full_mesh = np.moveaxis(a_glob_full_mesh, 0, 2)
                
                a = llc.substract_angles(a_glob_full_mesh, np.expand_dims(self.Y_ls, (0,1)))
                
                da = llc.substract_angles(a, landmark_param['a'])
                
                pY = norm.pdf(da, scale = landmark_param['sa'])
                
                if self.params['calc_type'] == "ADDITION":                            
                    self.s_grid += pY * self.params['yaw_discount']
                else:
                    self.s_grid *= pY * self.params['yaw_discount']
                    
    def merge_grids(self):
        if self.params['calc_type'] == "ADDITION":                                        
            self.m_grid = self.m_grid * self.params['prev_step_weight'] + self.s_grid * (1 - self.params['prev_step_weight'])
        else:
            self.m_grid = self.m_grid * self.s_grid
        
    def get_pose(self):        
        self.merge_grids()
        ipose = np.unravel_index(np.argmax(self.m_grid), self.m_grid.shape)        
        pose = []
        for i, p in enumerate(ipose):
            pose.append(list(self.params['dims'].values())[i]['min'] + list(self.params['dims'].values())[i]['res'] * p)                
        #self.m_grid = self.reset_grid()                
        self.cov = self.calc_cov(pose)
        return pose
    
    def calc_cov(self, pose):
        dx = self.xx_mg - pose[0]
        dy = self.yy_mg - pose[1]
        dX = np.array((dx.flatten(), dy.flatten()))
        dX = np.repeat(dX, self.params['dims']['Y']['size'], axis = 1)
        w = self.m_grid.flatten()
        w = w / np.sum(w)
        # TODO: add angle covs!
        return np.dot( dX * w , dX.T)
    
    def plot(self):
        plt.pcolor(self.plot_mx, self.plot_my, np.sum(self.m_grid, axis=2), cmap=plt.cm.get_cmap("Reds"), vmin = 0, edgecolors = 'k', alpha = 0.25)
        
if __name__ == '__main__': 
    pass
