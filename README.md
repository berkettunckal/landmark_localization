# landmark_localization

__== Project in early stage of development==__

ROS package for robot localization by landmarks.

## Included methods:
### 2D localization: 
 - [x] Basic Histogram filter (HF)
 - [ ] Density Trees Histogram Filter
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
dims: # filter area
 x: # dimention description
  d_res: # desired dimention resolution, final resolution may be different but not more
  min: # min dimetion value
  max: # max dimention value
 y: # same as x
 Y: # same as x and y
calc_type: 'ADDITION' # or can be 'MULTIPLICATION' - how to deal with probabilities 
yaw_discount: 1 # for discounting angle influence
prev_step_weight: 0.5 # how mush previous state is influencing on new one
motion_update_type: 'BLUR_SHIFT' # or can be 'PREV_COV' - model of previous pose integration
pose_calc_type: 'MAX' # or can be 'SUM' - type of calculating final robot pose
```
##### 1.1.1.2. ROS-wrapper [ll_hf2d_ros](src/landmark_localization/ll_hf2d_ros.py)
__Parameters:__
 - __~hf_params__ (dict, {}) method parameters as described below  

__Published topics:__
 - __~grid__ ([nav_msgs/OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)) grid visualization via costmap, works well when resolutions of x and y are same
#### 1.1.2. AMCL
##### 1.1.2.1. Implementation [ll_amcl2d](src/landmark_localization/ll_amcl2d.py)
Parameters dictionary can be imported from yaml-file:
```yaml
dims:
 x: 
  min:
  max:
 y: # same as x 
 Y: # same as x and y
NP: 100 # base particle number
NPmax: 100 # maximal number of particles
calc_type: "ADDITION" # or can be "MULTIPLICATION" - how to deal with probabilities 
alpha: [1,1,1,1,1,1] # motion parameters
alpha_slow: 0.0001 # 
alpha_fast: 0.1 #
```
##### 1.1.2.2. ROS-wrapper [ll_amcl2d_ros](src/landmark_localization/ll_amcl2d_ros.py)
__Parameters:__
- __~amcl_params__ (dict, {}) method parameters as described below
- __~publish_debug__ (bool, false) if true publishes `w_avg`, `w_slow` and `w_fast`.  

__Published topics:__
 - __~particles__ ([geometry_msgs/PoseArray](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseArray.html)) particles visualization
### 1.2. Sub Definite Localization addons on probabilistic methods
My own ideas of usage of Sub Definite Models in robot localization task to decrece search area for probabilistic methods.
Usage of SDL with HF is described in [Moscowsky, Anton. (2021). Subdefinite Computations for Reducing the Search Space in Mobile Robot Localization Task. 10.1007/978-3-030-86855-0_13.](https://www.researchgate.net/publication/355050502_Subdefinite_Computations_for_Reducing_the_Search_Space_in_Mobile_Robot_Localization_Task)
#### 1.2.1. Implementation [ll_sdl2d](src/landmark_localization/ll_sdl2d.py)
Parameters dictionary can be imported from yaml-file:
```yaml
dims:
 x: 
  min:
  max:
 y: # same as x 
 Y: # same as x and y
var_acc: 3 # for comparing variables when do subdef main process
stop_acc: 0.01 # 
verbose: true # print subdef process
max_steps: 100 # treshold of subdef steps
max_mc_rolls: 64 # max steps of MonteCarlo assigment function
use_correctness_check: true # do or not checking process on start of each iteration
inner_method: # 'hf' or 'amcl'
inner_method_params: {} # dict as described below for AMCL or HF
```
#### 1.2.2. ROS-wrapper [ll_sdl2d_ros](src/landmark_localization/ll_sdl2d_ros.py)
__Parameters:__
- __~sdl_params__ (dict, {}) method parameters as described below

__Published topics:__
- __~sd_areas__ ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)) subdef variables visualization
- __~amcl_particles__ ([geometry_msgs/PoseArray](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseArray.html)) particles visualization, if AMCL is choosen as inner method
- __~hf_grid__ ([nav_msgs/OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)) grid visualization if HF is choosen as inner method

## 2. Common ROS-interface for all methods


