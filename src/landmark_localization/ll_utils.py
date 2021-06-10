#!/usr/bin/env python
# coding: utf-8

import rospy
import numpy as np
import tf
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, TransformStamped

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
