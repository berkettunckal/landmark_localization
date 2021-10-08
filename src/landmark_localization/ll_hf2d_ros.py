#!/usr/bin/env python
# coding: utf-8

import rospy
from landmark_localization.landmark_localization_ros_2d import LandmarkLocalizationRos2D
from landmark_localization.ll_hf2d import HF2D
from nav_msgs.msg import OccupancyGrid
import numpy as np

class HFROS2D(LandmarkLocalizationRos2D):
    
    def __init__(self):        
        rospy.init_node('hf_2d_landmark_localization') 
        
        hf_params = rospy.get_param("~hf_params",{})        
        hf = HF2D(hf_params)        
        
        #self.publish_debug = rospy.get_param("~publish_debug", False)
        #if self.publish_debug:
            #self.debug_pub = rospy.Publisher("~debug", Float64MultiArray, queue_size = 1)
                        
        super(HFROS2D, self).__init__(hf)
        
        if self.visualizate_output:        
            self.vis_pub = rospy.Publisher("~grid", OccupancyGrid, queue_size = 1)
            
    def visualizate(self):        
        msg = get_grid_msg(self.ll_method, self.map_frame)
        self.vis_pub.publish(msg)

def get_grid_msg(hf_method, map_frame):
    msg = OccupancyGrid()
        
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = map_frame
        
    w = np.sum(hf_method.m_grid, axis = 2)
    w = w.T
    w = w.flatten()
    w = (w / np.max(w)) * 100
    msg.data = w.astype(np.int8).tolist()
    
    msg.info.map_load_time = rospy.Time.now()
    msg.info.resolution = hf_method.params['dims']['x']['res'] #NOTE: wrong if x and y has really different resolutions
    msg.info.width = hf_method.params['dims']['x']['size']
    msg.info.height = hf_method.params['dims']['y']['size']
    msg.info.origin.position.x = hf_method.params['dims']['x']['min']
    msg.info.origin.position.y = hf_method.params['dims']['y']['min']
    
    return msg
