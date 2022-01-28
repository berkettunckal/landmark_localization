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

#### 1.1.2. AMCL

### 1.2. Sub Definite Localization addons on probabilistic methods
My own ideas of usage of Sub Definite Models in robot localization task to decrece search area for probabilistic methods.
### SDL+HF
Usage of SDL with HF is described in [Moscowsky, Anton. (2021). Subdefinite Computations for Reducing the Search Space in Mobile Robot Localization Task. 10.1007/978-3-030-86855-0_13.](https://www.researchgate.net/publication/355050502_Subdefinite_Computations_for_Reducing_the_Search_Space_in_Mobile_Robot_Localization_Task)

### SDL+AMCL




