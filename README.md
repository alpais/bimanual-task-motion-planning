# Bi-manual task Motion Planning (Zucchini Peeling)
This repository includes the packages and instructions to run the LASA Bimanual Motion planning architecture developed initially for a bimanual zucchini peeling task within the Robohow project, but can be used for any bimanual task which invloves coordination between two end-effectors and controlling for a desired cartesian pose/ft/stiffness.

---
###Installation:

####System Requirements:

OS: Ubuntu 14.04

ROS compatibility: Indigo

####Instructions:

For **each package/repo** listed below, the user needs to do the following:

*Download:*
```
$ cd /catkin_ws/src
$ git clone <remote branch>
```
*Build:*
```
$ cd /catkin_ws/
$ catkin_make
```
####Package list:
  1. Install [robot-toolkit](https://github.com/epfl-lasa/robot-toolkit):
  ```
  $ git clone https://github.com/epfl-lasa/robot-toolkit.git
  ```

  2. Install [kuka_interface_packages](https://github.com/nbfigueroa/kuka_interface_packages):
  ```
  $ git clone https://github.com/nbfigueroa/kuka_interface_packages.git
  ```

  3. Install [kuka-rviz-simulation](https://github.com/epfl-lasa/kuka-rviz-simulation):
  ```
  $ git clone https://github.com/epfl-lasa/kuka-rviz-simulation.git
  ```
  and don't forget to install all [dependencies](https://github.com/epfl-lasa/kuka-rviz-simulation) for this package.

  4. Install [coupled-dynamical-systems](https://github.com/epfl-lasa/coupled-dynamical-systems) package:
  ```
  $ git clone https://github.com/epfl-lasa/coupled-dynamical-systems.git
  ```
  
  5. Install [state-transfomers](https://github.com/epfl-lasa/state-transformers) package:
  ```
  $ git clone https://github.com/epfl-lasa/state-transformers
  ```
  
  6. Finally, install [bimanual-task-motion-planning](https://github.com/epfl-lasa/bimanual-task-motion-planning) package:
  ```
  $ git clone https://github.com/epfl-lasa/bimanual-task-motion-planning.git
  ```
  
---
###Simulation of a Bi-manual reach with independent CDS for each arm:


---
###Real-Time Control of a Bi-manual reach with independent CDS for each arm:


#####Robot State Communication
Bringup ```kuka_fri_bridge``` (a custom KUKA control bridge using FRI library) check instructions to run [here](https://github.com/nbfigueroa/kuka_interface_packages.git).
```
$ rosrun kuka_fri_bridge run_lwr.sh
```
#####Real-time Robot Visualization
```
$ roslaunch kuka_lwr_bringup lwr_realtime_viz.launch
```

#####Control/Motion Planning
Cartesian-to-Joint/Joint-to-Cart Estimation
```
$ roslaunch state_transformers pouring_ctrls_real.launch
```

Cartesian Trajectory Generator
```
$ roslaunch motion_planner lasa_sim_fixed_pouring_tool.launch
```

##### Action Planning  
```
$ rosrun lasa_action_planners pouring_tool_demo_fixed_lasa.py
```
then follow the instructions on the terminal of this node.  

The robot will then follow the learned pouring trajectories: [Pouring Trajectories](https://www.dropbox.com/s/fgxrk9lj5avlw0j/pour_demo.mp4?dl=0)


--- 
###References:

[1] N. Figueroa and A. Billard, “Discovering hierarchical structure in heterogenous and sequential task demonstrations,” In preparation.

[2] A. L. Pais, K. Umezawa, Y. Nakamura, and A. Billard, “Task parametrization using continuous constraints extracted from human demonstrations,” Accepted, IEEE TRO, 2015.

[3] A. Shukla and A. Billard, “Coupled dynamical system based arm-hand grasping model for learning fast adaptation strategies,” Robotics and Autonomous Systems, vol. 60, no. 3, pp. 424 – 440, 2012.
