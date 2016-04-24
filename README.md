## Bi-manual Task Motion Planning
This repository includes the packages and instructions to run the LASA Bimanual Motion planning architecture developed initially for a bimanual zucchini peeling task within the Robohow project, but can be used for any bimanual task which invloves coordination between two end-effectors and controlling for a desired cartesian pose/ft/stiffness.

---

#### Requirements:

OS: Ubuntu 14.04
ROS compatibility: Indigo

  
| Dependencies  |
| ------------- |
| [kuka-rviz-simulation](https://github.com/epfl-lasa/kuka-rviz-simulation)           |
| [kuka_interface_packages](https://github.com/nbfigueroa/kuka_interface_packages)    |
| [coupled-dynamical-systems](https://github.com/epfl-lasa/coupled-dynamical-systems) |
| [state-transfomers](https://github.com/epfl-lasa/state-transformers)                |
| [bimanual-dynamical-system](https://github.com/epfl-lasa/bimanual-dynamical-system) |
  
---
###Simulation of a Bi-manual reach with independent CDS for each arm:

#####Simulation and Visualization
```
$ roslaunch kuka_lwr_bringup bimanual_simulation.launch
```

#####Low-level Controllers
Cartesian-to-Joint/Joint-to-Cart Estimation
```
$ roslaunch state_transformers bimanual_joint_ctrls_sim.launch 
```

#####Cartesian Trajectory Generator
```
$ roslaunch bimanual_motion_planner decoupled_test.launch
```

If you want to visualize the ideal trajectories generated by the models without moving the robots type the following:
```
$ roslaunch bimanual_motion_planner bimanual_test.launch simulation:=true
```

##### Action Planning  
```
$ rosrun bimanual_action_planners decoupled_test.py
```
then follow the instructions on the terminal of this node.  

---


