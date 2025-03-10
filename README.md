# Benchmark for Social Robotic Navigation

This repository contains a benchmark for robotic social navigation. An evaluation was made comparing several baselines from the literature according to different metrics, performance and social.
This benchmark is based on [SEAN](https://sean.interactive-machines.com), which is a simulator running under Unity3d allowing you to compare methods using metrics.
On this new version, we have added the possibility of testing navigation methods under ROS2 by upgrading the simulator to ROS2. We added metrics and offered six everyday life scenarios, crowded and sparse scenarios.

# Baselines

Here is the list of different baselines used for the comparison:

  - [DWB](https://github.com/ros-navigation/navigation2/blob/main/nav2_dwb_controller/README.md") 
  - [HATEB](https://github.com/sphanit/CoHAN_Navigation/tree/master)
  - [CADRL](https://github.com/mit-acl/cadrl_ros)
  - [SARL*](https://github.com/LeeKeyu/sarl_star)
  - [DRL_VO](https://github.com/TempleRAIL/drl_vo_nav)
  - [BRNE](https://github.com/MurpheyLab/brne)
  - Ours

# Installation

The experiments were conducted in Ubuntu 22.04 with ROS Humble and Unity 2022.3.10f1.

```
.                   # ~/social_benchmark_robotic_navigation
├── ros2_ws         # ROS2 Workspace, contains TCP_Connector ROS2-Unity and MBSN
├── tmux            # Tmux files to  launch different baseline on our unity simulator
└── sbrn            # Unity Project

```


## Dependencies

 . We recommend installing ```Tmux``` which is a terminal multiplexer and ```Tmuxinator``` which is a tool that allows you to easily manage tmux sessions using yaml files, we have provided different yaml files for each baseline in the folder ```~/social_benchmark_robotic_navigation/tmux/<navigation method>```. 

 . We used Docker to containerize all ROS1 baselines and used ros_bridge to be able to use our benchmark in ROS2. You can find the docker images [here](https://hub.docker.com/repository/docker/agouguet/benchmark-social-navigation/general).

 . For our method, it is enough to use the ros2 workspace present in this repository and for DWB to use nav2 from ROS2.

## Usage

For example, to use MBSN method :

```
cd ~/benchmark_social_navigation/tmux/MBSN
tmuxinator
```
