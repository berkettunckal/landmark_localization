#!/usr/bin/env python
# coding: utf-8

import rospy
from landmark_localization.landmark_localization_ros_2d import LandmarkLocalizationRos2D
from landmark_localization.ll_utils import quaternion_msg_from_yaw
from landmark_localization.ll_amcl2d import AMCL2D
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

class AMCLROS2D(LandmarkLocalizationRos2D):
    
    def __init__(self):        
        rospy.init_node('amcl_2d_landmark_localization') 
        
        amcl_params = rospy.get_param("~amcl_params",{})        
        amcl = AMCL2D(amcl_params)        
        
        self.publish_debug = rospy.get_param("~publish_debug", False)
        if self.publish_debug:
            self.debug_pub = rospy.Publisher("~debug", Float64MultiArray, queue_size = 1)
                        
        super(AMCLROS2D, self).__init__(amcl)
        if self.visualizate_output:
            self.vis_pub = rospy.Publisher("~particles", PoseArray, queue_size = 1)
                    
    def visualizate(self):
        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        for p in range(self.ll_method.P.shape[0]):
            pose = Pose()
            pose.position.x = self.ll_method.P[p,0]
            pose.position.y = self.ll_method.P[p,1]
            pose.orientation = quaternion_msg_from_yaw(self.ll_method.P[p,2])
            msg.poses.append(pose)
        self.vis_pub.publish(msg)
        
    def proc_cb(self, event):
        super(AMCLROS2D, self).proc_cb(event)        
        if self.publish_debug:
            msg = Float64MultiArray()            
            msg.data.append(self.ll_method.w_avg)
            d = MultiArrayDimension()
            d.label = "w_avg"
            msg.layout.dim.append(d)
            msg.data.append(self.ll_method.w_slow)
            d = MultiArrayDimension()
            d.label = "w_slow"
            msg.layout.dim.append(d)
            msg.data.append(self.ll_method.w_fast)            
            d = MultiArrayDimension()
            d.label = "w_fast"
            msg.layout.dim.append(d)
            self.debug_pub.publish(msg)        

if __name__ == '__main__': 
    amclros2d = AMCLROS2D()
    amclros2d.run()
    
