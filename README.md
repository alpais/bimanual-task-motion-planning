## Bi-manual Task Motion Planning (Zucchini Peeling)
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

  1. [kuka-rviz-simulation](https://github.com/epfl-lasa/kuka-rviz-simulation):
  ```
  $ git clone https://github.com/epfl-lasa/kuka-rviz-simulation.git
  ```
  and don't forget to install all [dependencies](https://github.com/epfl-lasa/kuka-rviz-simulation) for this package.
  
  2. If not already installed, [kuka_interface_packages](https://github.com/nbfigueroa/kuka_interface_packages):
  ```
  $ git clone https://github.com/nbfigueroa/kuka_interface_packages.git
  ```
  and don't forget to install all [dependencies](https://github.com/nbfigueroa/kuka_interface_packages) for this package.

  3. [coupled-dynamical-systems](https://github.com/epfl-lasa/coupled-dynamical-systems) package:
  ```
  $ git clone https://github.com/epfl-lasa/coupled-dynamical-systems.git
  ```
  
  4. [state-transfomers](https://github.com/epfl-lasa/state-transformers) package:
  ```
  $ git clone https://github.com/epfl-lasa/state-transformers
  ```
  
---
###Simulation of a Bi-manual reach with independent CDS for each arm:

#####Simulation and Visualization
```
$ roslaunch kuka_lwr_bringup bimanual_simulation.launch
```

#####Control/Motion Planning
Cartesian-to-Joint/Joint-to-Cart Estimation
```
$ roslaunch state_transformers bimanual_joint_ctrls_sim.launch 
```

Cartesian Trajectory Generator
```
$ roslaunch bimanual_motion_planner bimanual_test.launch
```

##### Action Planning  
```
$ rosrun bimanual_action_planner bimanual_test.py
```
then follow the instructions on the terminal of this node.  

---
###Real-Time Control of a Bi-manual reach with independent CDS for each arm:

#####Robot State Communication
Bringup ```kuka_fri_bridge``` (a custom KUKA control bridge using FRI library) check instructions to run [here](https://github.com/nbfigueroa/kuka_interface_packages.git).

For each robot:

Modifying the following line:
```
$ ...
```
and run the bridge on its corresponding pc:
```
$ rosrun kuka_fri_bridge run_lwr.sh
```
#####Real-time Robot Visualization
```
$ roslaunch kuka_lwr_bringup bimanual_realtime_viz.launch
```


--- 
###References:

[1] N. Figueroa and A. Billard, “Discovering hierarchical structure in heterogenous and sequential task demonstrations,” In preparation.

[2] A. L. Pais, K. Umezawa, Y. Nakamura, and A. Billard, “Task parametrization using continuous constraints extracted from human demonstrations,” Accepted, IEEE TRO, 2015.

[3] A. Shukla and A. Billard, “Coupled dynamical system based arm-hand grasping model for learning fast adaptation strategies,” Robotics and Autonomous Systems, vol. 60, no. 3, pp. 424 – 440, 2012.
