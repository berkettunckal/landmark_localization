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
#from scipy.spatial.distance import mahalanobis
from scipy.stats import multivariate_normal

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
        motion_update_type: BLUR_SHIFT (def) | PREV_COV
        pose_calc_type: MAX (def) | SUM
        
    '''
    def __init__(self, params = {}):
        super(HF2D, self).__init__()        
        
        if not 'calc_type' in params or (params['calc_type'] != "ADDITION" and params['calc_type'] != "MULTIPLICATION"):
            params['calc_type'] = "ADDITION"                    
        if not 'yaw_discount' in params:
            params['yaw_discount'] = 1
        if not 'prev_step_weight' in params:
            params['prev_step_weight'] = 0.5
        if not 'motion_update_type' in params or (params['motion_update_type'] != 'BLUR_SHIFT' and params['motion_update_type'] != 'PREV_COV'):
            params['motion_update_type'] = 'BLUR_SHIFT'
        if not 'pose_calc_type' in params:
            params['pose_calc_type'] = 'MAX'
                            
        # TODO check other params
        self.params = params 
        
        self.reset()
        
    def reset(self):
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
        
        '''
        self.plot_mx, self.plot_my = np.mgrid[slice(self.params['dims']['x']['min'],
                                self.params['dims']['x']['min'] + self.params['dims']['x']['res'] * (self.params['dims']['x']['size']),
                                self.params['dims']['x']['res']),
                                    slice(self.params['dims']['y']['min'],                                
                                self.params['dims']['y']['min'] + self.params['dims']['y']['res'] * (self.params['dims']['y']['size']),
                                self.params['dims']['y']['res'])]                                            
        '''
        self.plot_mx, self.plot_my = np.mgrid[slice(self.params['dims']['x']['min'],
                                self.params['dims']['x']['max'],
                                self.params['dims']['x']['res']),
                                    slice(self.params['dims']['y']['min'],                                
                                self.params['dims']['y']['max'],
                                self.params['dims']['y']['res'])]  
                                    
        self.plot_mx = self.plot_mx[:self.grid_size[0], :self.grid_size[1]]
        self.plot_my = self.plot_my[:self.grid_size[0], :self.grid_size[1]]
                                                        
        self.prev_pose = None
        
    def get_linspace(self, dim):
        return np.linspace(dim['min'], dim['max'], dim['size'])                
        
    def reset_grid(self):
        #if self.params['calc_type'] == "ADDITION":            
            #grid = np.zeros(self.grid_size)
        #else:
            #grid = np.ones(self.grid_size)
        grid = np.ones(self.grid_size)
        grid = grid / grid.size
        return grid
                    
    def count_dim(self, dim):
        dim_len = dim['max'] - dim['min']
        if dim_len > 0:
            dim['size'] = int(np.ceil(dim_len / dim['d_res']))
            dim['res'] = dim_len / dim['size']
            dim['max']  += dim['res']
            dim['size'] += 1
        else:
            dim['min_len'] -= dim['d_res']/2
            dim['max_len'] += dim['d_res']/2
            dim['size'] = 1
            dim['res'] = dim['d_res']
        
    
    def motion_update(self, motion_params):
        # TODO super check params
                
        r = motion_params['dt'] * motion_params['vx']
        da = motion_params['dt'] * motion_params['wY']
                
        if self.params['motion_update_type'] == 'BLUR_SHIFT':
            # TODO stash motion between steps without landmark update
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
        elif self.params['motion_update_type'] == 'PREV_COV':
            
            if not self.prev_pose is None:                
                self.m_grid = self.reset_grid()                                
                                               
                shifted_Y = (self.prev_pose[2] + da + np.pi)%(2*np.pi) - np.pi
                shifted_x = self.prev_pose[0] + r * np.cos(shifted_Y)
                shifted_y = self.prev_pose[1] + r * np.sin(shifted_Y)
                
                shifted = np.array([shifted_x, shifted_y, shifted_Y])
                
                #def plot_robot_pose(x, y, Y, color, label = None):
                    #plt.plot(x, y, "o", color = color)
                    #plt.plot([x, x + np.cos(Y)], [y, y + np.sin(Y)], "-", color = color, label = label)                                    
                #plot_robot_pose(shifted_x, shifted_y, shifted_Y, 'cyan', 'shifted')
               
                X = self.get_X().T
                
                dX = np.zeros(X.shape)
                dX[:,:2] = X[:,:2] - shifted[:2]
                dX[:,2] =  llc.substract_angles(X[:,2], shifted[2])                
                
                #print(self.cov)
                p = multivariate_normal.pdf(dX, mean = np.zeros(shifted.shape), cov = self.cov, allow_singular = True)
                
                p = p.reshape((self.m_grid.shape[1], self.m_grid.shape[0], self.m_grid.shape[2])).swapaxes(0,1)
                
                if self.params['calc_type'] == "ADDITION":                                                                                    
                    self.m_grid += p
                else:
                    self.m_grid *= p
            
        self.s_grid = self.reset_grid()
    
    # NOTE that function could be more vectorized
    def landmarks_update(self, landmarks_params ):
        #TODO super check params
        if len(landmarks_params) == 0:
            return
            
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
        self.s_grid = self.s_grid / len(landmarks_params)
                    
    def merge_grids(self):
        if self.params['calc_type'] == "ADDITION":                                        
            self.m_grid = self.m_grid / np.sum(self.m_grid)
            self.s_grid = self.s_grid / np.sum(self.s_grid)
            self.m_grid = self.m_grid * self.params['prev_step_weight'] + self.s_grid * (1 - self.params['prev_step_weight'])
        else:
            self.m_grid = self.m_grid / np.sum(self.m_grid)
            self.s_grid = self.s_grid / np.sum(self.s_grid)
            self.m_grid = self.m_grid * self.s_grid# NOTE: prev weight has no sense?
        
    def get_pose(self):        
        self.merge_grids()
        self.weight = np.max(self.m_grid)
        if self.params['pose_calc_type'] == 'MAX':            
            ipose = np.unravel_index(np.argmax(self.m_grid), self.m_grid.shape)        
            pose = []
            for i, p in enumerate(ipose):
                pose.append(list(self.params['dims'].values())[i]['min'] + list(self.params['dims'].values())[i]['res'] * p)
        elif self.params['pose_calc_type'] == 'SUM':
            X = self.get_X()            
            w = np.swapaxes(self.m_grid, 0, 1)
            w = w.flatten()
            w = w / np.sum(w)        
            pose = [np.sum(X[0,:] * w),
                    np.sum(X[1,:] * w),
                    np.sum(X[2,:] * w)]
            
        self.cov = self.calc_cov(pose)
        self.prev_pose = pose
        return pose
    
    def calc_cov(self, pose):
        w = np.swapaxes(self.m_grid, 0, 1)
        w = w.flatten()
        w = w / np.sum(w)                
        dX = self.get_all_d(pose)                
        return np.dot( dX * w , dX.T)
    
    def get_all_d(self, pose):
        dx = self.xx_mg - pose[0]
        dy = self.yy_mg - pose[1]        
        dY = llc.substract_angles(self.Y_ls, pose[2])        
        dxy = np.array(( dx.flatten(), dy.flatten() ))
        dxy = np.repeat(dxy, self.params['dims']['Y']['size'], axis = 1)        
        dYY = np.tile(dY, self.x_ls.shape[0] * self.y_ls.shape[0])                
        dX = np.vstack((dxy, dYY))                
        return dX
    
    def get_X(self):
        # TODO not nessesary to calc it again and again
        xy = np.array(( self.xx_mg.flatten(), self.yy_mg.flatten() ))
        xy = np.repeat(xy, self.params['dims']['Y']['size'], axis = 1)
        YY = np.tile(self.Y_ls, self.x_ls.shape[0] * self.y_ls.shape[0])
        X = np.vstack((xy, YY))        
        return X
            
    def plot(self):                
        plt.pcolor(self.plot_mx, self.plot_my, np.sum(self.m_grid, axis=2), cmap=plt.cm.get_cmap("Reds"), vmin = 0, edgecolors = 'k', alpha = 0.25, shading='nearest')
        
if __name__ == '__main__': 
    pass
