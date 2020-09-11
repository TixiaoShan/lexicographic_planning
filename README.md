# Lexicographic Planning

This package includes a planning algorithm that utilizes lexicographic optimization for solving guided-planning problems.

## Dependency

Install ROS and [jackal_velodyne](https://github.com/TixiaoShan/jackal_velodyne) package for a minimum demo. It is strongly recommended that you upgrade your Gazebo to get better simulation performance.

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with Kinetic and Melodic)
  ```
  sudo apt-get install -y ros-kinetic-navigation
  ```
- [jackal_velodyne](https://github.com/TixiaoShan/jackal_velodyne)
  ```
  sudo apt-get install ros-kinetic-jackal-*
  sudo apt-get install ros-kinetic-velodyne-*
  cd ~/catkin_ws/src
  git clone https://github.com/TixiaoShan/jackal_velodyne.git
  cd ..
  catkin_make
  ```
- [Upgrade Gazebo](https://github.com/TixiaoShan/jackal_velodyne#upgrade-gazebo)
  ```
  sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
  wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
  sudo apt update
  sudo apt upgrade
  ```

## Install

  ```
  cd ~/catkin_ws/src
  git clone https://github.com/TixiaoShan/lexicographic_planning.git
  cd ..
  catkin_make
  ```

## Testing

Using the following command to start running the package. If this is the first time you run Gazebo on your computer, it may take a while to launch because Gazebo needs to download some resources.
  ```
  roslaunch lexicographic_planning run.launch
  ```
Click **2D Nav Goal** in **Rviz**, then click anywhere in the window. The robot should start following the path.

<p align='center'>
    <img src="./config/demo.gif" alt="drawing" width="800"/>
</p>

## Parameters

 - The number of costs that are optimized is defined by **NUM_COSTS** in ```utility.h```.
 - The type of costs are defined in **edgePropagation()** function in ```pathPlanning.cpp```.
 - More parameters can be found in ```params.yaml```.

## Paper 

Thank you for citing [our paper](https://arxiv.org/abs/2007.08362) if you use any of this code. 
```
@inproceedings{shan2020receding,
  title={A Receding Horizon Multi-Objective Planner for Autonomous Surface Vehicles in Urban Waterways},
  author={Shan, Tixiao and Wang, Wei and Englot, Brendan and Ratti, Carlo and Rus Daniela},
  booktitle={59th IEEE Conference on Decision and Control (CDC)}
  year={2020},
  organization={IEEE}
}
```