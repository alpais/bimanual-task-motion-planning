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
$ roslaunch kuka_lwr_bringup bimanual2_simulation.launch
```

#####Low-level Controllers
Cartesian-to-Joint/Joint-to-Cart Estimation
```
$ roslaunch state_transformers bimanual_joint_ctrls_sim.launch 
```

#####Cartesian Trajectory Generator
A bimanual action server, containing different types of control methods for bimanual action, currently 1) independent CDS for each arm (no spatial coupling) and 2) Virtual Object Dynamical System (spatial and temporal coupling)

```
$ roslaunch bimanual_motion_planner bimanual_action_server.launch
```

##### Action Planning  

To run a test with two CDS models independently for each arm:
```
$ rosrun bimanual_action_planners decoupled_test.py
```

To run a test with the Virtual Object Dynamical System:
```
$ rosrun bimanual_action_planners virtual_object_test.py
```

---


