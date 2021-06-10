#!/usr/bin/env python
# coding: utf-8

import rospy
from landmark_localization.landmark_localization_ros_2d import LandmarkLocalizationRos2D
from landmark_localization.ll_hf2d import HF2D

class HFROS2D(LandmarkLocalizationRos2D):
    
    def __init__(self):        
        rospy.init_node('hf_2d_landmark_localization') 
        
        hf_params = rospy.get_param("~hf_params",{})        
        hf = HF2D(hf_params)        
        
        #self.publish_debug = rospy.get_param("~publish_debug", False)
        #if self.publish_debug:
            #self.debug_pub = rospy.Publisher("~debug", Float64MultiArray, queue_size = 1)
                        
        super(HFROS2D, self).__init__(hf)
        #if self.visualizate_output:        
        
if __name__ == '__main__': 
    hfros2d = HFROS2D()
    hfros2d.run()
