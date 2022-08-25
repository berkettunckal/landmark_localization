#!/usr/bin/env python
# coding: utf-8

'''
Module that uses SDL to find area of robot by visual landmarks and ask LIDAR AMCL to reinit when its position is outside of found area
'''
import rospy
from landmark_localization.landmark_localization_ros_2d import LandmarkLocalizationRos2D
from landmark_localization.ll_sdl2d import SDL2D
from landmark_localization.ll_sdl2d_ros import get_markers_msg
from visualization_msgs.msg import MarkerArray

class SDL_LIDAR_ROS(LandmarkLocalizationRos2D):
    
    def __init__(self):
        rospy.init_node('sdl_lidar_ros') 
        
        sdl_params = rospy.get_param("~sdl_params",{})
        rospy.logwarn(sdl_params)
        sdl = SDL2D(sdl_params)
        
        super(SDL_LIDAR_ROS, self).__init__(sdl)
        
        if self.visualizate_output:
            self.prev_markers_num = 0
            self.vis_pub = rospy.Publisher("~sd_areas", MarkerArray, queue_size = 1)
        
    def proc_cb(self, event):
        current_time = rospy.Time.now()
        # NOTE check odom time? but if it not passed proove what to do?
        if not self.last_odom_msg is None:
            mp = self.odom_msg_to_motion_params(self.last_odom_msg)                
            self.ll_method.motion_update(mp)     
            self.last_odom_msg = None
        
        id_list = []
        if not self.last_landmark_msg is None:
            if (current_time - self.last_landmark_msg.header.stamp).to_sec() <= self.max_time_lag:
                lp = self.eod_msg_to_landmarks_params(self.last_landmark_msg, self.eod_id_value, self.used_eod_ids, self.used_map_ids)
                if not lp is None:
                    self.ll_method.landmarks_update(lp)
                    if self.visualizate_map:
                        id_list = self.get_id_list_from_eod_msg(self.last_landmark_msg, self.eod_id_value)
                        self.last_landmark_msg = None
            else:
                rospy.logwarn("[{}] skipped landmark data due to old timestamp.".format(rospy.get_name()))
            
        self.ll_method.get_pose()
        
        if self.visualizate_output:
            self.visualizate()
                    
        if self.visualizate_map:
            #print(id_list)
            self.map_pub.publish(self.landmark_map.return_as_marker_array(current_time, self.map_frame, id_list))
            
    def visualizate(self):
        msg = get_markers_msg(self.ll_method, self.map_frame, self.prev_markers_num)
        self.prev_markers_num = len(msg.markers)
        self.vis_pub.publish(msg)
        
