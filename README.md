# improved_mpl

## Abstract

Search-based motion planning with motion primitives is a complete and optimal method, and it can generate dynamically feasible, collision-free, smooth trajectories in dense environments. 

To improve the computational efficiency of this method, we focus on the motion primitive graph search method and collision detection method, and propose three improvements: 

- design an admissible heuristic function that considers environmental obstacles

- propose two motion primitive cost penalty strategies

- propose a novel collision detection method with linear complexity based on Sturm's Theory In structured obstacle environments

## Installation

### Running Environment:

- ubuntu 20

- ros noetic

- cmake: minimum 3.21.0

### Prerequisite Package:

- eigen, yaml, opencv, sdl

```
sudo apt install -y libeigen3-dev libyaml-cpp-dev libopencv-dev libsdl1.2-dev libsdl-image1.2-dev
```

- kd tree: 

[nanoflann](https://link.zhihu.com/?target=https%3A//github.com/jlblancoc/nanoflann)

## Usage

### Build and Complile

```
git clone --recursive https://github.com/ucas-zihaowang/improved_mpl.git
cd  improved_mpl
catkin_make -DCMAKE_BUILD_TYPE=release
source devel/setup.bash
```

### Example:

- parameter: in  src/slplanner_plus/config/parameter_xxx.yaml

- running for different instances

```
cd improved_mpl
roslaunch slplanner_plus test_slplanner_demo_by_corridor.launch
roslaunch slplanner_plus test_slplanner_demo_by_corridor_penalty.launch
roslaunch slplanner_plus test_slplanner_demo_by_corridor_penalty.launch
roslaunch slplanner_plus test_slplanner_demo_by_corridor_with_yaw.launch
roslaunch slplanner_plus test_slplanner_demo_by_corridor_with_yaw_penalty.launch
```

- others
experiment_demo: can visualize the instances by rviz
slplanner_plus: different map
    test_slplanner_demo_by_clcbsbenchmark.launch
    test_slplanner_demo_by_tmpmap.launch


## Acknowledgment

Our project refers to the following work, and we point out their contributions here.

[motion_primitive_library](https://github.com/sikang/motion_primitive_library.git)

[mpl_ros](https://github.com/sikang/mpl_ros.git)
