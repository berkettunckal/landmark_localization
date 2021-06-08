#!/usr/bin/env python
# coding: utf-8

import rospy
from landmark_localization.landmark_localization_ros_2d import LandmarkLocalizationRos2D
from landmark_localization.ll_amcl2d import AMCL2D

class AMCLROS2D(LandmarkLocalizationRos2D):
    
    def __init__(self):        
        rospy.init_node('amcl_2d_landmark_localization') 
        
        amcl_params = rospy.get_param("~amcl_params",{})        
        amcl = AMCL2D(amcl_params)        
        super(AMCLROS2D, self).__init__(amcl)


if __name__ == '__main__': 
    amclros2d = AMCLROS2D()
    amclros2d.run()
    
