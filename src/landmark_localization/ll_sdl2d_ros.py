#!/usr/bin/env python
# coding: utf-8

import rospy
from landmark_localization.landmark_localization_ros_2d import LandmarkLocalizationRos2D
from landmark_localization.ll_sdl2d import SDL2D
from nav_msgs.msg import OccupancyGrid
import numpy as np
from landmark_localization.ll_amcl2d_ros import get_particles_msg
from geometry_msgs.msg import PoseArray

class SDLROS2D(LandmarkLocalizationRos2D):
    
    def __init__(self):        
        rospy.init_node('sdl_2d_landmark_localization') 
        
        sdl_params = rospy.get_param("~sdl_params",{})
        sdl = SDL2D(sdl_params)
        
        super(SDLROS2D, self).__init__(sdl)
        
        if self.visualizate_output:
            if sdl.params['inner_method'] == 'amcl':
                self.amcl_particles_pub = rospy.Publisher("~amcl_particles", PoseArray, queue_size = 1)
        
    def visualizate(self):
        if self.ll_method.params['inner_method'] == 'amcl':
            msg = get_particles_msg(self.ll_method.inner_amcl, self.map_frame)
            self.amcl_particles_pub.publish(msg)
