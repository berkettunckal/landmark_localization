#!/usr/bin/env python
# coding: utf-8

import rospy
from landmark_localization.landmark_localization_ros_2d import LandmarkLocalizationRos2D
from landmark_localization.ll_sdl2d import SDL2D
from nav_msgs.msg import OccupancyGrid
import numpy as np
from landmark_localization.ll_amcl2d_ros import get_particles_msg
from landmark_localization.ll_hf2d_ros import get_grid_msg
from geometry_msgs.msg import PoseArray, Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

class SDLROS2D(LandmarkLocalizationRos2D):
    
    def __init__(self):        
        rospy.init_node('sdl_2d_landmark_localization') 
        
        sdl_params = rospy.get_param("~sdl_params",{})
        sdl = SDL2D(sdl_params)
        
        super(SDLROS2D, self).__init__(sdl)
        
        if self.visualizate_output:
            self.prev_markers_num = 0
            self.vis_pub = rospy.Publisher("~sd_areas", MarkerArray, queue_size = 1)
            if sdl.params['inner_method'] == 'amcl':
                self.amcl_particles_pub = rospy.Publisher("~amcl_particles", PoseArray, queue_size = 1)
            elif sdl.params['inner_method'] == 'hf':
                self.hf_grid_pub = rospy.Publisher("~hf_grid", OccupancyGrid, queue_size = 1)
            
            
    def visualizate(self):
        msg = get_markers_msg(self.ll_method, self.map_frame, self.prev_markers_num)
        self.prev_markers_num = len(msg.markers)
        self.vis_pub.publish(msg)
        
        if self.ll_method.params['inner_method'] == 'amcl':
            msg = get_particles_msg(self.ll_method.inner_amcl, self.map_frame)
            self.amcl_particles_pub.publish(msg)
            
        elif self.ll_method.params['inner_method'] == 'hf':
            msg = get_grid_msg(self.ll_method.inner_hfs[0] , self.map_frame)
            self.hf_grid_pub.publish(msg)
            
            
def get_markers_msg(sdl_method, map_frame, previous_marker_num = 0):
    msg = MarkerArray()
    
    counter = 0
    for x in sdl_method.model.variables['x']['VALUE']:
        for y in sdl_method.model.variables['y']['VALUE']:
            
            m = Marker()
            m.header.frame_id = map_frame
            m.header.stamp = rospy.Time.now()
            m.id = counter
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
                        
            m.scale.x = 0.01            
            m.pose.orientation.w = 1
            m.color.r = 1
            m.color.b = 1
            m.color.a = 1
            
            m.points.append(Point(x.low, y.low, 0))
            m.points.append(Point(x.low, y.high, 0))
            m.points.append(Point(x.high, y.high, 0))
            m.points.append(Point(x.high, y.low, 0))
            m.points.append(Point(x.low, y.low, 0))
            
            counter+=1
            msg.markers.append(m)
            for Y in sdl_method.model.variables['Y']['VALUE']:
                m = Marker()
                m.header.frame_id = map_frame
                m.header.stamp = rospy.Time.now()
                m.id = counter
                m.action = Marker.ADD
                m.type = Marker.LINE_LIST                
                
                m.scale.x = 0.01            
                m.pose.orientation.w = 1
                m.color.r = 1
                m.color.b = 1
                m.color.a = 1
                
                xc = (x.low + x.high)/2.
                yc = (y.low + y.high)/2.
                m.points.append(Point(xc, yc, 0))
                m.points.append(Point(xc + 0.5*np.cos(Y.low), yc + 0.5*np.sin(Y.low), 0))
                m.points.append(Point(xc, yc, 0))
                m.points.append(Point(xc + 0.5*np.cos(Y.high), yc + 0.5*np.sin(Y.high), 0))
                
                counter+=1
                msg.markers.append(m)
                
                m = Marker()
                m.header.frame_id = map_frame
                m.header.stamp = rospy.Time.now()
                m.id = counter
                m.action = Marker.ADD
                m.type = Marker.LINE_STRIP
                
                m.scale.x = 0.01            
                m.pose.orientation.w = 1
                m.color.r = 1
                m.color.b = 1
                m.color.a = 1
                
                #for yi in range(Y.low, Y.high, 0.5):
                yi = Y.low
                while yi < Y.high:
                    m.points.append(Point(xc + 0.25*np.cos(yi), yc + 0.25*np.sin(yi), 0))
                    yi+= 0.5
                m.points.append(Point(xc + 0.25*np.cos(Y.high), yc + 0.25*np.sin(Y.high), 0))
                
                counter+=1
                msg.markers.append(m)
            
    for i in range(counter, previous_marker_num):
        m = Marker()
        m.header.frame_id = map_frame
        m.header.stamp = rospy.Time.now()                
        m.id = i
        m.action = Marker.DELETE
        msg.markers.append(m)
    
    return msg
                
    
    
