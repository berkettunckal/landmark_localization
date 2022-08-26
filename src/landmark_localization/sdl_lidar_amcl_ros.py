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
from landmark_localization.ll_utils import yaw_from_quaternion_msg, quaternion_msg_from_yaw
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped

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
            
        self.relocaliaze_amcl_pub = rospy.Publisher('amcl/initialpose', PoseWithCovarianceStamped, queue_size = 1)
        
    def proc_cb(self, event):
                
        current_time = rospy.Time.now()
        
        # 1. motion update
        # NOTE check odom time? but if it not passed proove what to do?
        if not self.last_odom_msg is None:
            mp = self.odom_msg_to_motion_params(self.last_odom_msg)                
            self.ll_method.motion_update(mp)     
            self.last_odom_msg = None
        
        # 2. landmark update
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
        
        # 3. calculate area
        self.ll_method.get_pose()
        
        # 4. get AMCL pose
        try:
            map_bl_tf = self.tf_buffer.lookup_transform(self.map_frame,
                                                        self.base_frame,
                                                        rospy.Time(0),
                                                        rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            rospy.logerr('[{}] timed out transform {} to {}'.format(rospy.get_name(), self.map_frame, self.base_frame))
            return
        
        # 5. check pose is in area
        if( map_bl_tf.transform.translation.x in self.ll_method.model.variables['x']['VALUE'] and
            map_bl_tf.transform.translation.y in self.ll_method.model.variables['y']['VALUE'] and
            yaw_from_quaternion_msg(map_bl_tf.transform.rotation) in self.ll_method.model.variables['Y']['VALUE'] ):
            ## 5.1. pose in - do noting
            rospy.loginfo("Robot in SDL zone");            
        else:                        
            ## 5.2 pose out - make relocalize pose by this area for AMCL
            rospy.logwarn("Robot outside of SDL zone!")
            
            # get highest value for each vars
            max_x = self.ll_method.model.variables['x']['VALUE'].max_module()
            max_y = self.ll_method.model.variables['y']['VALUE'].max_module()
            max_Y = self.ll_method.model.variables['Y']['VALUE'].max_module()
            
            r_pose = PoseWithCovarianceStamped()
            r_pose.header.frame_id = self.map_frame
            r_pose.header.stamp = rospy.Time.now()
            
            r_pose.pose.pose.position.x = self.ll_method.model.variables['x']['VALUE'][max_x[1]].center()
            
            r_pose.pose.pose.position.y = self.ll_method.model.variables['y']['VALUE'][max_y[1]].center()
            
            r_pose.pose.pose.orientation = quaternion_msg_from_yaw(self.ll_method.model.variables['Y']['VALUE'][max_Y[1]].center())
            
            r_pose.pose.covariance[0] = max_x[0]/6 # xx
            r_pose.pose.covariance[7] = max_y[0]/6 # yy
            r_pose.pose.covariance[35] = max_Y[0]/6 # YY
                    
            self.relocaliaze_amcl_pub.publish(r_pose)
        
        if self.visualizate_output:
            self.visualizate()
                    
        if self.visualizate_map:
            #print(id_list)
            self.map_pub.publish(self.landmark_map.return_as_marker_array(current_time, self.map_frame, id_list))
            
    
            
    def visualizate(self):
        msg = get_markers_msg(self.ll_method, self.map_frame, self.prev_markers_num)
        self.prev_markers_num = len(msg.markers)
        self.vis_pub.publish(msg)
        
