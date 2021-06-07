#!/usr/bin/env python
# coding: utf-8
import yaml

class LandmarkMap(object):
    def __init__(self):
        self.map = {}
        
    def load(self, path):
        try:
            with open(path, 'r') as f:
                self.map = yaml.load(f, Loader = yaml.FullLoader)
                
                # NOTE 2d simple check
                if type(self.map) is dict:
                    for k, l in self.map.items():
                        if not (type(k) is int or type(k) is str):
                            #raise ValueError("MapFormatError: id key must be int or string")
                            print("MapFormatError: id key must be int or string")
                            return False
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
    
    def return_as_marker(self):
        raise NotImplementedError('save')


if __name__ == '__main__':
    map_file = "../../test_data/test_map.yaml"
    LM = LandmarkMap()
    LM.load(map_file)
    print(LM)
    
        
