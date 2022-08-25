#!/usr/bin/env python
# coding: utf-8

from landmark_localization.sdl_lidar_amcl_ros import SDL_LIDAR_ROS
                
if __name__ == '__main__': 
    sdl_amcl_ros = SDL_LIDAR_ROS()
    sdl_amcl_ros.run()
