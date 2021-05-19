#!/usr/bin/env python
# coding: utf-8

'''
Adaptive Monte-Carlo Localization for landmark localization task
'''
import landmark_localization.landmark_localization_core as llc

class AMCL(llc.LandmarkLocalization):
    
    '''
    amcl_params
        NP - particle amount
    '''
    def __init__(self, amcl_params = {}):
        super(AMCL, self).__init__()
        
        self.parmas = amcl_params
        
        
if __name__ == '__main__': 
    pass
