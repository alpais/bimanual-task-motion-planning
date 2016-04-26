## Bi-manual Task Motion Planning
This repository includes the packages and instructions to run the LASA Bimanual Motion planning architecture developed initially for a bimanual zucchini peeling task within the Robohow project, but can be used for any bimanual task which invloves coordination between two end-effectors and controlling for a desired cartesian pose/ft/stiffness.

---

#### Requirements:

OS: Ubuntu 14.04
ROS compatibility: Indigo

---
###Simulation of a Bi-manual reaching motions:

| Dependencies  |
| ------------- |
| [kuka-rviz-simulation](https://github.com/epfl-lasa/kuka-rviz-simulation)           |
| [kuka_interface_packages](https://github.com/nbfigueroa/kuka_interface_packages)    |
| [coupled-dynamical-systems](https://github.com/epfl-lasa/coupled-dynamical-systems) |
| [state-transfomers](https://github.com/epfl-lasa/state-transformers)                |
| [bimanual-dynamical-system](https://github.com/epfl-lasa/bimanual-dynamical-system) |

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
A bimanual action server, containing different types of control methods for bimanual action, currently 1) independent CDS for each arm (no coupling) and 2) Virtual Object Dynamical System (spatial and temporal coupling)

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
###Demonstration Recording for Bimanual Tasks:

To record/replay demonstrations you must install these additional packages:

| Dependencies  |
| ------------- |
| [net-ft-ros](https://github.com/epfl-lasa/net-ft-ros) |
| [vision_pkg](https://github.com/epfl-lasa/vision_pkg) |
| [record_ros](https://github.com/epfl-lasa/record_ros) |

#####Stream Robot data (joint states, Pose, FT, Stiff)
```
$ rosrun kuka_fri_bridge run_lwr.sh right
```
```
$ rosrun kuka_fri_bridge run_lwr.sh left
```

#####Visualization and sensor bringup (ft sensors, vision)
```
$ roslaunch kuka_lwr_bringup bimanual2_realtime.launch ft_sensors:=true vision:=true 
```

##### Recorder node for all topics necessary
```
$ roslaunch bimanual_action_planners record_bimanual_demos.launch 
```

##### Start/Stop a Recording (Rosbag)
```
$ rosservice call /record/cmd "cmd: 'record/stop'"
```
---
###Replaying a recorded demonstration of a Bimanual Task:
#####Visualization and sensor bringup (ft sensors, vision)
```
$ roslaunch kuka_lwr_bringup bimanual2_realtime.launch ft_sensors:=true not_bag:=false 
```
#####Play bag
```
$ rosbag play *.bag
```

