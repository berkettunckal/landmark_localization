#!/usr/bin/env python
# coding: utf-8

import rospy
import numpy as np
import tf
import tf2_ros

from nav_msgs.msg import Odometry
from extended_object_detection.msg import SimpleObjectArray, ComplexObjectArray
from landmark_localization.landmark_map import LandmarkMap
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, TransformStamped
from visualization_msgs.msg import MarkerArray
import landmark_localization.ll_utils as ll_utils

class LandmarkLocalizationRos2D(object):
    def __init__(self, ll_method, ignore_map = False):        
        '''
        MAP
        '''
        if not ignore_map:
            map_path = rospy.get_param('~map_path', None)        
            self.landmark_map = LandmarkMap()
            if not self.landmark_map.load(map_path):
                rospy.logerr("Map {} not found, exit".format(map_path))
                exit()        
            
        '''
        localization method, should be one inherited from LandmarkLocalization (landmark_localization_ros_2d.py)
        '''
        self.ll_method = ll_method
        '''
        other variables
        '''
        self.prev_odom_time = rospy.Time.now()
        self.landmark_r_sigma = rospy.get_param('~landmark_r_sigma', 0.1)
        self.landmark_a_sigma = rospy.get_param('~landmark_a_sigma', 0.1)
        
        self.last_landmark_msg = None
        self.last_complex_landmark_msg = None
        self.last_odom_msg = None
        
        self.eod_id_value = rospy.get_param('~eod_id_value')
        self.used_eod_ids = rospy.get_param('~used_eod_ids', [])
        if not ignore_map:
            self.used_map_ids = rospy.get_param('~used_map_ids', self.landmark_map.get_ids())
        
        self.landmark_transform = None
        self.landmark_target_frame = rospy.get_param('~landmark_target_frame', None) # WHAT IT IS FOR?
        if self.landmark_target_frame is None:
            self.landmark_transform = np.eye(4)
            
        
        self.max_time_lag = rospy.get_param('~max_time_lag', 0.5)
        self.publish_tf = rospy.get_param('~publish_tf', True)
        
        self.add_tf_time = rospy.Duration(rospy.get_param('~add_tf_time', 0))
        
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        
        # p ps pc pcs o
        self.output_data_format = set(rospy.get_param('~output_data_format', ['ps']))
        df_to_msg = {'p': Pose,
                     'ps': PoseStamped,
                     'pc': PoseWithCovariance,
                     'pcs': PoseWithCovarianceStamped,
                     'o': Odometry}        
        self.output_pubs = {}
        for df in self.output_data_format:
            self.output_pubs[df] = rospy.Publisher('~robot_pose_{}'.format(df), df_to_msg[df], queue_size = 1)
        
        # tf initialization
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        if self.landmark_transform is None or self.publish_tf:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # visualization
        self.visualizate_output = rospy.get_param("~visualizate_output", True)
        self.visualizate_map = rospy.get_param("~visualizate_map", True)
        
        if self.visualizate_map:
            self.map_pub = rospy.Publisher("~landmark_map", MarkerArray, queue_size = 1)
        
        proc_rate = rospy.get_param('proc_rate_hz', 5.0)        
                        
        rospy.Subscriber('odom', Odometry, self.odom_cb)
        rospy.Subscriber('eod', SimpleObjectArray, self.eod_cb)
        rospy.Subscriber('eod_complex', ComplexObjectArray, self.eod_complex_cb)
        rospy.Timer( rospy.Duration(1.0/proc_rate), self.proc_cb)
    
    '''
    converts Odometry msg to motion params
    '''
    def odom_msg_to_motion_params(self, msg):
        # NOTE: now only for 2d
        motion_params = {}
        motion_params['dt'] = (rospy.Time.now() - self.prev_odom_time).to_sec()
        self.prev_odom_time = rospy.Time.now()
        motion_params['vx'] = msg.twist.twist.linear.x
        motion_params['svx'] = msg.twist.covariance[0]
        motion_params['wY'] = msg.twist.twist.angular.z
        motion_params['swY'] = msg.twist.covariance[35]
        return motion_params                
    
    '''
    converts SimpleObjectArray to landmarks_params
        ids - list of type_ids\type_names, should be used for landmark data, if empty all is used
        sub_ids - list of sub_ids should be used for landmark data, if empty all is used
        transform - 4x4 matrix of required transformation (commonly cam_frame -> base_link) 
    '''
    def eod_msg_to_landmarks_params(self, msg, id_value, ids = [], sub_ids = []):
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
        if type(msg) == SimpleObjectArray:
            for so in msg.objects:
                landmark_param = self.base_obj_to_landmark_param(so, id_value, ids)
                if landmark_param is not None:
                    landmarks_params.append(landmark_param)
        elif type(msg) == ComplexObjectArray:
            for co in msg.objects:
                landmark_param = self.base_obj_to_landmark_param(co.complex_object, id_value, ids)
                if landmark_param is not None:
                    landmarks_params.append(landmark_param)
                for so in co.simple_objects:
                    landmark_param = self.base_obj_to_landmark_param(so, id_value, ids)
                    if landmark_param is not None:
                        landmarks_params.append(landmark_param)
                            
        #rospy.logwarn(landmarks_params)
        return landmarks_params   
    
    def base_obj_to_landmark_param(self, so, id_value, ids):
        if len(ids) == 0 or so.type_id in ids or so.type_name in ids:
            id_dict = dict(zip(so.extracted_info.keys, so.extracted_info.values))                
                
            id_criteria = id_value in id_dict and int(id_dict[id_value]) in self.landmark_map
            name_criteria = so.type_name in self.landmark_map
                
            if id_criteria or name_criteria:
                landmark_param = {}
                
                if id_criteria:
                    index = int(id_dict[id_value])
                if name_criteria:
                    index = so.type_name
                    
                landmark_param['x'] = self.landmark_map[index]['x']
                landmark_param['y'] = self.landmark_map[index]['y']
                
                vector_src = np.array([[so.transform.translation.x, so.transform.translation.y, so.transform.translation.z, 0]]).T                                        
                vector_dst = np.dot(self.landmark_transform, vector_src)                    
                
                if so.transform.translation.z != 1:# z = 1 means real transform is't known
                    landmark_param['r'] = float(np.hypot(vector_dst[0], vector_dst[1]))
                    landmark_param['sr'] = self.landmark_r_sigma
                landmark_param['a'] = float(np.arctan2(-vector_dst[1], -vector_dst[0])) #NOTE: not sure it is good
                landmark_param['sa'] = self.landmark_a_sigma
                
                return landmark_param
        return None        
        
    def get_id_list_from_eod_msg(self, msg, id_value):
        ids = []
        for so in msg.objects:
            id_dict = dict(zip(so.extracted_info.keys, so.extracted_info.values))            
            if id_value in id_dict:
                ids.append(int(id_dict[id_value]))
        return ids                    
    
    def odom_cb(self, msg):
        self.last_odom_msg = msg
    
    def eod_cb(self, msg):
        self.last_landmark_msg = msg
    
    def eod_complex_cb(self, msg):
        self.last_complex_landmark_msg = msg
    
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
                lp += self.eod_msg_to_landmarks_params(self.last_complex_landmark_msg, self.eod_id_value, self.used_eod_ids, self.used_map_ids)
                #rospy.logwarn(lp)
                if not lp is None:
                    self.ll_method.landmarks_update(lp)
                    if self.visualizate_map:
                        id_list = self.get_id_list_from_eod_msg(self.last_landmark_msg, self.eod_id_value)
                        self.last_landmark_msg = None
            else:
                rospy.logwarn("[{}] skipped landmark data due to old timestamp.".format(rospy.get_name()))
            
        robot_pose = self.ll_method.get_pose()
        
        out_msg_p = ll_utils.robot_pose_to_pose(robot_pose)
        if any(item in self.output_data_format for item in ['pc', 'pcs', 'o']):
            cov = self.ll_method.get_cov()
            if cov is None:
                cov = np.zeros((3,3))
        for df, msg_t in self.output_pubs.items():
            if df == 'p':
                self.output_pubs[df].publish(out_msg_p)
            if df == 'ps':
                self.output_pubs[df].publish(ll_utils.robot_pose_to_pose_stamped(robot_pose, self.map_frame))
            if df == 'pc':
                self.output_pubs[df].publish(ll_utils.robot_pose_to_pose_with_covariance(robot_pose, cov))
            if df == 'pcs':
                self.output_pubs[df].publish(ll_utils.robot_pose_to_pose_with_covariance_stamped(robot_pose, cov, self.map_frame))
            if df == 'o':
                self.output_pubs[df].publish(ll_utils.robot_pose_to_odometry(robot_pose, cov, self.map_frame, self.odom_frame, [0,0], []))       
                
        if self.publish_tf:
            self.do_publish_tf(robot_pose, current_time)
            
        if self.visualizate_output:
            self.visualizate()
                    
        if self.visualizate_map:
            #print(id_list)
            self.map_pub.publish(self.landmark_map.return_as_marker_array(current_time, self.map_frame, id_list))
                        
    # broadcasts odom from map as correction to new base position
    def do_publish_tf(self, robot_pose, ts = None):
        try:    
            odom_bl_tf = self.tf_buffer.lookup_transform(self.odom_frame, 
                                                         self.base_frame,
                                                         rospy.Time(0), 
                                                         rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
            rospy.logerr('[{}] timed out transform {} to {}'.format(rospy.get_name(), self.odom_frame, self.base_frame))
            return
        
        odom_bl_mat = tf.transformations.compose_matrix(
            translate = [odom_bl_tf.transform.translation.x,
                         odom_bl_tf.transform.translation.y,
                         odom_bl_tf.transform.translation.z],
            angles = ll_utils.euler_from_quaternion_msg(odom_bl_tf.transform.rotation))
        
        map_bl_mat = tf.transformations.compose_matrix(
            translate = [robot_pose[0],robot_pose[1], 0.],
            angles = [0., 0., robot_pose[2]])
                
        map_odom_mat = np.dot(map_bl_mat, np.linalg.inv(odom_bl_mat))
        
        _, _, angles, trans, _ = tf.transformations.decompose_matrix(map_odom_mat)
        
        tfs = TransformStamped()
        
        if ts is None:
            ts = rospy.Time.now()
        tfs.header.stamp = ts + self.add_tf_time
        tfs.header.frame_id = self.map_frame
        tfs.child_frame_id = self.odom_frame
        tfs.transform.translation.x = trans[0]
        tfs.transform.translation.y = trans[1]
        tfs.transform.translation.z = trans[2]
        tfs.transform.rotation = ll_utils.quaternion_msg_from_euler(angles)
        
        self.tf_broadcaster.sendTransform(tfs)
        
    def run(self):
        rospy.spin()
        
    def visualizate(self):
        raise NotImplementedError('visualizate')


