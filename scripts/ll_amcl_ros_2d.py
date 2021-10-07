#!/usr/bin/env python
# coding: utf-8
from landmark_localization.ll_amcl2d_ros import AMCLROS2D

if __name__ == '__main__': 
    amclros2d = AMCLROS2D()
    amclros2d.run()
    
