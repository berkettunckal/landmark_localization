#!/usr/bin/env python
# coding: utf-8

'''
landmark_localization is ROS-independed parent class for all localization methods
'''
import numpy as np

class LandmarkLocalization(object):
    
    def __init__(self):
        pass
        
    '''
    motion_params - dict:
        dt - time interval
        vx, vy, vz - linear velocities (in robot frame), for 2d only vx, vy (if holonomic) is used 
        svx, svy, svz - sigmas of linear velocities
        wR, wP, wY - angular velocities (in robot frame), for 2d only wY is used
        swR, swP, swY - sigmas of angular velocities
    dt, vx, svx, wY, swY - are nessesary for 2d localization of non-holonomic robot
    '''
    def motion_update(self, motion_params = {}):
        # TODO check
        raise NotImplementedError('motion_update')
        
    '''
    landmarks_params - list of dicts
        dict:
            MAP DATA
            x, y, z - position of landmark (in map frame), for 2d only x and y are used
            R, P, Y - orientation of landmark (in map frame), for 2d only Y is used
            MEASURMENTS
            r - distance to landmark (in robot main frame)
            sr - sigma of r  measuarment
            a, b, g - angles to landmark (in robot frame), for 2d only a is used 
            sa, sb, sg - sigmas of angle measurments
            lR, lP, lY - orientation of landmark, for 2d only lY is used
            slR, slP, slY - sigmas of orientation measurments
    x, y, (z) - nessesary params for 2d (3d) localization
    r OR a (b, g) and its sigmas - nessesary params for 2d (3d) localization
    others are optional
    '''
    def landmarks_update(self, landmarks_params = []):
        # TODO check
        raise NotImplementedError('landmark_update')
    
    '''
    global_update - list of dict
        dict:
            x, y, z - global position in map frame. for 2d only x is used
            sx, sy, sz - sigmas of position
            R, P, Y - global orienttaion in map frame, for 2d only Y is used                    
            sR, sP, sY - sigmas of orienttaion            
    x or Y (and its sigmas) are nessesary params for 2d
    '''
    def global_update(self, global_params = []):
        # TODO check
        raise NotImplementedError('global_update')
    
    '''
    returns:
        [x, y, Y]
    '''
    def get_pose(self):
        raise NotImplementedError('get_pose')
    
    def plot(self):
        raise NotImplementedError('plot')
    
def substract_angles(target, source):
    return np.arctan2(np.sin(target-source), np.cos(target-source))

def norm_angle(value):
    return (value + np.pi) % (2*np.pi) - np.pi

if __name__ == "__main__":
    pass
