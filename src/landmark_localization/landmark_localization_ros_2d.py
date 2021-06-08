#!/usr/bin/env python
# coding: utf-8

import rospy
import numpy as np
import tf
import tf2_ros

from nav_msgs.msg import Odometry
from extended_object_detection.msg import SimpleObjectArray
from landmark_localization.landmark_map import LandmarkMap
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, TransformStamped

class LandmarkLocalizationRos2D(object):
    def __init__(self, ll_method):        
        '''
        MAP
        '''
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
        self.last_odom_msg = None
        
        self.used_eod_ids = rospy.get_param('~used_eod_ids', [])
        self.used_map_ids = rospy.get_param('~used_map_ids', self.landmark_map.get_ids())
        
        self.landmark_transform = None
        self.landmark_target_frame = rospy.get_param('~landmark_target_frame', None)
        if self.landmark_target_frame is None:
            self.landmark_transform = np.eye(4)
            
        
        self.max_time_lag = rospy.get_param('~max_time_lag', 0.5)
        self.publish_tf = rospy.get_param('~publish_tf', True)
        
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
        
        proc_rate = rospy.get_param('proc_rate_hz', 5.0)        
        
        rospy.Subscriber('odom', Odometry, self.odom_cb)
        rospy.Subscriber('eod', SimpleObjectArray, self.eod_cb)
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
    def eod_msg_to_landmarks_params(self, msg, ids = [], sub_ids = [], transform = np.eye(4)):
        #if self.landmark_transform is None:
            
        landmarks_params = []
        for so in msg.objects:
            if len(ids) == 0 or so.type_id in ids or so.type_names in ids:
                if so.extracted_info[0].sub_id in self.landmark_map:
                    landmark_param = {}
                    landmark_param['x'] = self.landmark_map[so.extracted_info[0].sub_id]['x']
                    landmark_param['y'] = self.landmark_map[so.extracted_info[0].sub_id]['y']
                    
                    vector_src = np.array([[so.transform.translation.x, so.transform.translation.y, so.transform.translation.z, 0]]).T                                        
                    vector_dst = np.dot(transform, vector_src)
                    
                    if so.transform.translation.z != 1:# z = 1 means real transform is't known
                        landmark_param['r'] = np.hypot(vector_dst[0], vector_dst[1])
                        landmark_param['sr'] = self.landmark_r_sigma
                    landmark_param['a'] = np.arctan2(vector_dst[1], vector_dst[0])
                    landmark_param['sa'] = self.landmark_a_sigma
                    
                    landmarks_params.append(landmark_param)
        return landmarks_params                
        
    def odom_cb(self, msg):
        self.last_odom_msg = msg
    
    def eod_cb(self, msg):
        self.last_landmark_msg = msg
    
    def proc_cb(self, event):
        # NOTE check odom time? but if it not passed proove what to do?
        if not self.last_odom_msg is None:
            mp = self.odom_msg_to_motion_params(self.last_odom_msg)                
            self.ll_method.motion_update(mp)     
            self.last_odom_msg = None
        
        if not self.last_landmark_msg is None:
            if (rospy.Time.now() - self.last_landmark_msg.header.stamp).to_sec() <= self.max_time_lag:
                lp = self.eod_msg_to_landmarks_params(self.last_landmark_msg, self.used_eod_ids, self.used_map_ids)#, self.landmark_transform)
                self.ll_method.landmarks_update(lp)
            else:
                rospy.logwarn("[{}] skipped landmark data due to old timestamp.".format(rospy.get_name()))
            
        robot_pose = self.ll_method.get_pose()
        
        out_msg_p = robot_pose_to_pose(robot_pose)
        if any(item in self.output_data_format for item in ['pc', 'pcs', 'o']):
            cov = self.ll_method.get_cov()
        for df, msg_t in self.output_pubs.items():
            if df == 'p':
                self.output_pubs[df].publish(out_msg_p)
            if df == 'ps':
                self.output_pubs[df].publish(robot_pose_to_pose_stamped(robot_pose, self.odom_frame))
            if df == 'pc':
                self.output_pubs[df].publish(robot_pose_to_pose_with_covariance(robot_pose, cov))
            if df == 'pcs':
                self.output_pubs[df].publish(robot_pose_to_pose_with_covariance_stamped(robot_pose, cov, self.odom_frame))
            if df == 'o':
                self.output_pubs[df].publish(robot_pose_to_pose_with_covariance_stamped(robot_pose, cov, self.odom_frame, self.base_frame, [0,0], []))       
                
        if self.publish_tf:
            self.do_publish_tf(robot_pose)
                        
    # broadcasts odom from map as correction to new base position
    def do_publish_tf(self, robot_pose):
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
            angles = euler_from_quaternion_msg(odom_bl_tf.transform.rotation))
        
        map_bl_mat = tf.transformations.compose_matrix(
            translate = [robot_pose[0],robot_pose[1], 0.],
            angles = [0., 0., robot_pose[2]])
        
        map_odom_mat = np.dot(np.linalg.inv(odom_bl_mat), map_bl_mat) 
        
        _, _, angles, trans, _ = tf.transformations.decompose_matrix(map_odom_mat)
        
        tfs = TransformStamped()
        
        tfs.header.stamp = rospy.Time.now()
        tfs.header.frame_id = self.map_frame
        tfs.child_frame_id = self.odom_frame
        tfs.transform.translation.x = trans[0]
        tfs.transform.translation.y = trans[1]
        tfs.transform.translation.z = trans[2]
        tfs.transform.rotation = quaternion_msg_from_euler(angles)
        
        self.tf_broadcaster.sendTransform(tfs)
        
    def run(self):
        rospy.spin()

'''
common output of localization modules to Pose msg
    robot_pose - [x, y, Y]
'''
def robot_pose_to_pose(robot_pose):     
    pose = Pose()
    pose.position.x = robot_pose[0]
    pose.position.y = robot_pose[1]        
    pose.orientation = quaternion_msg_from_yaw(robot_pose[2])
    return pose

def robot_pose_to_pose_stamped(robot_pose, frame_id, stamp = None):
    if stamp is None:
        stamp = rospy.Time.now()
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose = robot_pose_to_pose(robot_pose)
    return pose

def robot_pose_to_pose_with_covariance(robot_pose, cov_mat):
    pose = PoseWithCovariance()
    pose.pose = robot_pose_to_pose(robot_pose)
    cov = np.zeros(36)
    cov[0] = cov_mat[0,0] # xx
    cov[1] = cov_mat[0,1] # xy
    cov[5] = cov_mat[0,2] # x alpha
    cov[6] = cov_mat[1,0] # yx
    cov[7] = cov_mat[1,1] # yy
    cov[11] = cov_mat[1,2] # y alpha
    cov[30] = cov_mat[2,0] # alpha x
    cov[31] = cov_mat[2,1] # alpha y
    cov[35] = cov_mat[2,2] # alpha alpha
    pose.covariance = list(cov)
    return pose

def robot_pose_to_pose_with_covariance_stamped(robot_pose, cov_mat, frame_id, stamp = None):
    if stamp is None:
        stamp = rospy.Time.now()
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose = robot_pose_to_pose_with_covariance(robot_pose, cov_mat)
    return pose

def robot_pose_to_odometry(robot_pose, cov_mat_pose, frame_id, child_frame_id, robot_speed, cov_mat_speed, stamp = None):
    if stamp is None:
        stamp = rospy.Time.now()
    odom = Odometry()
    odom.header.frame_id = frame_id
    odom.header.stamp = stamp
    odom.child_frame_id = child_frame_id
    odom.pose = robot_pose_to_pose_with_covariance(robot_pose, cov_mat_pose)
    odom.twist.twist.linear.x = robot_speed[0]
    odom.twist.twist.angular.z = robot_speed[1]
    odom_cov = np.zeros(36)
    odom_cov[0]  = cov_mat_speed[0]   # v v
    odom_cov[5]  = cov_mat_speed[0,2] # x w
    odom_cov[30] = cov_mat_speed[2,0] # w x
    odom_cov[35] = cov_mat_speed[2,2] # w w
    odom.covariance = list(odom_cov)
    
def yaw_from_quaternion_msg(q_msg):
    qu = []
    qu.append(q_msg.x)
    qu.append(q_msg.y)
    qu.append(q_msg.z)
    qu.append(q_msg.w)

    yaw = tf.transformations.euler_from_quaternion(qu)[2]
    return yaw

def euler_from_quaternion_msg(q_msg):
    qu = []
    qu.append(q_msg.x)
    qu.append(q_msg.y)
    qu.append(q_msg.z)
    qu.append(q_msg.w)

    return tf.transformations.euler_from_quaternion(qu)    

def quaternion_msg_from_yaw(yaw):
    qu = tf.transformations.quaternion_from_euler(0,0,yaw)    
    msg = Quaternion()
    msg.x = qu[0]
    msg.y = qu[1]
    msg.z = qu[2]
    msg.w = qu[3]
    return msg

def quaternion_msg_from_euler(euler):
    qu = tf.transformations.quaternion_from_euler(euler[0],euler[1],euler[2])    
    msg = Quaternion()
    msg.x = qu[0]
    msg.y = qu[1]
    msg.z = qu[2]
    msg.w = qu[3]
    return msg
