#!/usr/bin/env python
# coding: utf-8

'''
Module that uses SDL to find area of robot by visual landmarks and ask LIDAR AMCL to reinit when its position is outside of found area
Module is used with scene recognition of EOD and don't need any map, because EOD has it and already sends all need info about it
'''
import rospy
from landmark_localization.landmark_localization_ros_2d import LandmarkLocalizationRos2D
from landmark_localization.ll_sdl2d import SDL2D
from landmark_localization.ll_sdl2d_ros import get_markers_msg
from visualization_msgs.msg import MarkerArray
from landmark_localization.ll_utils import yaw_from_quaternion_msg, quaternion_msg_from_yaw
import tf2_ros
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from extended_object_detection.msg import SceneArray
import landmark_localization.ll_utils as ll_utils
import numpy as np

class SDL_LIDAR_ROS(LandmarkLocalizationRos2D):
    
    def __init__(self):
        rospy.init_node('sdl_lidar_ros') 
        
        sdl_params = rospy.get_param("~sdl_params",{})
        rospy.logwarn(sdl_params)
        sdl = SDL2D(sdl_params)
        
        super(SDL_LIDAR_ROS, self).__init__(sdl, ignore_map = True)
        
        if self.visualizate_output:
            self.prev_markers_num = 0
            self.vis_pub = rospy.Publisher("~sd_areas", MarkerArray, queue_size = 1)
            
        self.reloc_amcl_pub = rospy.Publisher('amcl/initialpose', PoseWithCovarianceStamped, queue_size = 1)
        
        self.last_scene = None
        rospy.Subscriber('eod_scene', SceneArray, self.eod_scene_cb)
        
    def eod_scene_cb(self, msg):
        self.last_scene = msg
        
    def eod_scene_to_landmarks(self, msg):
        if self.landmark_transform is None:
            try:    
                bl_cam_tf = self.tf_buffer.lookup_transform(self.base_frame,
                                                            msg.header.frame_id, 
                                                            rospy.Time(0), 
                                                            rospy.Duration(0.1))
                
                self.landmark_transform = tf.transformations.compose_matrix(
                    translate = [bl_cam_tf.transform.translation.x,
                                 bl_cam_tf.transform.translation.y,
                                 bl_cam_tf.transform.translation.z],
                    angles = ll_utils.euler_from_quaternion_msg(bl_cam_tf.transform.rotation))
                                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
                rospy.logerr('[{}] timed out transform {} to {}, can\'t proceed landmark data'.format(rospy.get_name(), self.base_frame, msg.header.frame_id))
                return None         
            
        landmarks_params = []
        
        # for scene in msg.scenes:
        # NOTE: do stuff only when there is only one scene detected
        if len(msg.scenes) == 1:
            for obj in msg.scenes[0].objects:
                landmark_param = {}
                landmark_param['x'] = obj.map_pose.x
                landmark_param['y'] = obj.map_pose.y
                
                vector_src = np.array([[obj.detected_object.transform.translation.x, obj.detected_object.transform.translation.y, obj.detected_object.transform.translation.z, 0]]).T                                        
                vector_dst = np.dot(self.landmark_transform, vector_src)                    
                
                if obj.detected_object.transform.translation.z != 1:# z = 1 means real transform is't known
                    landmark_param['r'] = float(np.hypot(vector_dst[0], vector_dst[1]))
                    landmark_param['sr'] = self.landmark_r_sigma
                landmark_param['a'] = float(np.arctan2(-vector_dst[1], -vector_dst[0])) #NOTE: not sure it is good
                landmark_param['sa'] = self.landmark_a_sigma
                
                landmarks_params.append(landmark_param)
                
        return landmarks_params            
        
        
    def get_amcl_pose(self):
        try:
            map_bl_tf = self.tf_buffer.lookup_transform(self.map_frame,
                                                        self.base_frame,
                                                        rospy.Time(0),
                                                        rospy.Duration(0.1))
            return map_bl_tf
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            rospy.logerr('[{}] timed out transform {} to {}'.format(rospy.get_name(), self.map_frame, self.base_frame))
            return None
        
    def check_pose_in_sdl_area(self, map_bl_tf):
        return map_bl_tf.transform.translation.x in self.ll_method.model.variables['x']['VALUE'] and map_bl_tf.transform.translation.y in self.ll_method.model.variables['y']['VALUE'] and          yaw_from_quaternion_msg(map_bl_tf.transform.rotation) in self.ll_method.model.variables['Y']['VALUE']
        
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
        if not self.last_scene is None:
            if (current_time - self.last_scene.header.stamp).to_sec() <= self.max_time_lag:
                lp = self.eod_scene_to_landmarks(self.last_scene)
                rospy.logwarn(lp)
                if not lp is None:
                    self.ll_method.landmarks_update(lp)
                    if self.visualizate_map:
                        id_list = []#self.get_id_list_from_eod_msg(self.last_landmark_msg, self.eod_id_value)
                        self.last_scene = None
            else:
                rospy.logwarn("[{}] skipped landmark data due to old timestamp.".format(rospy.get_name()))
        
        # 3. calculate area
        self.ll_method.get_pose()
        
        # 4. get AMCL pose
        map_bl_tf = self.get_amcl_pose()
        if map_bl_tf is None:
            return
        
        # 5. check pose is in area
        if( self.check_pose_in_sdl_area(map_bl_tf) ):
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
                    
            self.reloc_amcl_pub.publish(r_pose)
            
            # 6. Check relocalized result?
            rospy.sleep(0.05)
            map_bl_tf = self.get_amcl_pose()
            if not map_bl_tf is None:            
                if not self.check_pose_in_sdl_area(map_bl_tf):
                    rospy.logwarn("Robot still outside of SDL zone! Reseting SDL vars...")
                    self.ll_method.reset_main_vars()
                    
        
        # 7. Analyze cov of pose?
        
        
        if self.visualizate_output:
            self.visualizate()
                                        
    
            
    def visualizate(self):
        msg = get_markers_msg(self.ll_method, self.map_frame, self.prev_markers_num)
        self.prev_markers_num = len(msg.markers)
        self.vis_pub.publish(msg)
        
