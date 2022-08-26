#!/usr/bin/env python
# coding: utf-8
import yaml
from visualization_msgs.msg import MarkerArray, Marker
from landmark_localization.ll_utils import quaternion_msg_from_yaw

class LandmarkMap(object):
    def __init__(self):
        self.map = {}
        
    def load(self, path):
        try:
            self.ids = {}
            id = 0
            with open(path, 'r') as f:
                self.map = yaml.load(f, Loader = yaml.FullLoader)
                
                # NOTE 2d simple check
                if type(self.map) is dict:
                    for k, l in self.map.items():
                        if not (type(k) is int or type(k) is str):
                            #raise ValueError("MapFormatError: id key must be int or string")
                            print("MapFormatError: id key must be int or string")
                            return False
                        if not k in self.ids:
                            self.ids[k] = id
                            id+=1                            
                        if type(l) is dict:                            
                            if not 'x' in l:
                                #raise ValueError("MapFormatError: x key not found in landmark data")
                                print("MapFormatError: x key not found in landmark data")
                                return False
                            if not 'y' in l:
                                #raise ValueError("MapFormatError: y key not found in landmark data")                                
                                print("MapFormatError: y key not found in landmark data")
                                return False
                        else:
                            #raise ValueError("MapFormatError: all landmark data should be stored as dicts")                            
                            print("MapFormatError: all landmark data should be stored as dicts")          
                            return False
                else:
                    #raise ValueError("MapFormatError: map should be stored in file as dict")
                    print("MapFormatError: map should be stored in file as dict")
                    return False
                return True
        
        except OSError:
            print("Map file {} not found!".format(path))
            return False
        
    def __str__(self):
        return self.map.__str__()
        
    def __contains__(self, item):
        return item in self.map
    
    def __getitem__(self, indices):
        return self.map[indices]      
    
    def get_ids(self):
        return list(self.map.keys())
        
    def save(self, path):
        # NOTE: for future
        raise NotImplementedError('save')
    
    def plot(self):
        raise NotImplementedError('save')
    
    # TODO: more params
    def return_as_marker_array(self, stamp, frame_id, visible = []):        
        map_msg = MarkerArray()
        for id, pose in self.map.items():
            marker_msg = Marker()
            marker_msg.header.frame_id = frame_id
            marker_msg.header.stamp = stamp
            
            marker_msg.id = self.ids[id]
            marker_msg.ns = "marker"
            
            marker_msg.type = Marker.CUBE
            marker_msg.action = Marker.ADD
            
            marker_msg.pose.position.x = pose['x']
            marker_msg.pose.position.y = pose['y']
            if 'z' in pose:
                marker_msg.pose.position.z = pose['z']
            else:
                marker_msg.pose.position.z = 0.3
            
            marker_msg.pose.orientation = quaternion_msg_from_yaw(pose['Y'])
            
            marker_msg.scale.x = 0.185
            marker_msg.scale.y = 0.01
            marker_msg.scale.z = 0.185
            
            if id in visible:
                marker_msg.color.r = 1
                marker_msg.color.g = 0
                marker_msg.color.b = 0
                marker_msg.color.a = 1
            else:
                marker_msg.color.r = 1
                marker_msg.color.g = 1
                marker_msg.color.b = 1
                marker_msg.color.a = 1
            
            map_msg.markers.append(marker_msg)
            
            text_msg = Marker()
            text_msg.header.frame_id = frame_id
            text_msg.header.stamp = stamp
            
            text_msg.id = self.ids[id]
            text_msg.ns = "text"
            
            text_msg.type = Marker.TEXT_VIEW_FACING
            text_msg.action = Marker.ADD
            
            text_msg.pose.position.x = pose['x']
            text_msg.pose.position.y = pose['y']
            if 'z' in pose:
                text_msg.pose.position.z = pose['z'] + 0.2
            else:
                text_msg.pose.position.z = 0.5
            
            text_msg.pose.orientation.w = 1
            text_msg.scale.z = 0.1
            
            if id in visible:
                text_msg.color.r = 1
                text_msg.color.g = 0
                text_msg.color.b = 0
                text_msg.color.a = 1
            else:
                text_msg.color.r = 1
                text_msg.color.g = 1
                text_msg.color.b = 1
                text_msg.color.a = 1
                
            text_msg.text = str(id)
            
            map_msg.markers.append(text_msg)
            
        return map_msg
            


if __name__ == '__main__':
    map_file = "../../test_data/test_map.yaml"
    LM = LandmarkMap()
    LM.load(map_file)
    print(LM)
    
        
