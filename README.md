# landmark_localization

__== Project in early stage of development==__

ROS package for robot localization by landmarks.

## Included methods:
### 2D localization: 
 - [x] Histogram filter (HF)
 - [x] Augmented Monte Carlo Localization (AMCL)
 - [x] Sub Definite Localiztion (SDL) with inner HF
 - [x] Sub Definite Localiztion (SDL) with inner AMCL
 - [ ] Extended Kalman Filter (EKF)

## 1. Methods Description
### 1.1. Pure probabilistic methods
Methods represented in [Thrun, Sebastian, Burgard, Wolfram and Fox, Dieter. Probabilistic robotics. Cambridge, Mass.: MIT Press, 2005.](https://docs.ufpr.br/~danielsantos/ProbabilisticRobotics.pdf).
#### 1.1.1. Histogram filter
##### 1.1.1.1. Implementation [ll_hf2d](src/landmark_localization/ll_hf2d.py)
Parameters dictionary can be imported from yaml-file:
```yaml
dims:
 x: 
  d_res:
  min:
  max:
 y: 
  d_res:
  min:
  max:
 Y:
  d_res:
  min:
  max:
calc_type: 'ADDITION' # 'MULTIPLICATION'
yaw_discount: 1
prev_step_weight: 0.5
motion_update_type: 'BLUR_SHIFT' # 'PREV_COV'
pose_calc_type: 'MAX' # 'SUM'
```
##### 1.1.1.2. ROS-wrapper [ll_hf2d_ros](src/landmark_localization/ll_hf2d_ros.py)
__Parameters:__
 - __~hf_params__ (dict, {}) method parameters as described below  
__Published topics:__
 - __~grid__ ([nav_msgs/OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)) grid visualization via costmap, works well when resolutions of x and y are same
#### 1.1.2. AMCL
##### 1.1.1.1. Implementation [ll_amcl2d](src/landmark_localization/ll_amcl2d.py)
Parameters dictionary can be imported from yaml-file:
```yaml
dims:
 x: 
  d_res:
  min:
  max:
 y: 
  d_res:
  min:
  max:
 Y:
  d_res:
  min:
  max:
NP: 100
NPmax: 100
calc_type: "ADDITION" # "MULTIPLICATION"
alpha: [1,1,1,1,1,1]
alpha_slow: 0.0001
alpha_fast: 0.1
```
##### 1.1.1.2. ROS-wrapper [ll_amcl2d_ros](src/landmark_localization/ll_amcl2d_ros.py)
__Parameters:__
- __~amcl_params__ (dict, {}) method parameters as described below
- __~publish_debug__ (bool, false) if true publishes `w_avg`, `w_slow` and `w_fast`.  
__Published topics:__
 - __~particles__ ([geometry_msgs/PoseArray](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseArray.html)) particles visualization
### 1.2. Sub Definite Localization addons on probabilistic methods
My own ideas of usage of Sub Definite Models in robot localization task to decrece search area for probabilistic methods.
Usage of SDL with HF is described in [Moscowsky, Anton. (2021). Subdefinite Computations for Reducing the Search Space in Mobile Robot Localization Task. 10.1007/978-3-030-86855-0_13.](https://www.researchgate.net/publication/355050502_Subdefinite_Computations_for_Reducing_the_Search_Space_in_Mobile_Robot_Localization_Task)
#### 1.2.1. Implementation [ll_sdl2d](src/landmark_localization/ll_sdl2d.py)
#### 1.2.2. ROS-wrapper [ll_sdl2d_ros](src/landmark_localization/ll_sdl2d_ros.py)





