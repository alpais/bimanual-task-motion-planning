## Bi-manual Task Motion Planning
This repository includes the packages and instructions to run the LASA Bimanual Motion planning architecture developed initially for a bimanual zucchini peeling task within the Robohow project, but can be used for any bimanual task which invloves coordination between two end-effectors and controlling for a desired cartesian pose/ft/stiffness.

---

#### Requirements:

OS: Ubuntu 14.04
ROS compatibility: Indigo

| Dependencies  |
| ------------- |
| [kuka_interface_packages](https://github.com/nbfigueroa/kuka_interface_packages)    |
| [kuka-rviz-simulation](https://github.com/epfl-lasa/kuka-rviz-simulation)           |
| [net-ft-ros](https://github.com/epfl-lasa/net-ft-ros) |
| [coupled-dynamical-systems](https://github.com/epfl-lasa/coupled-dynamical-systems) |
| [state-transfomers](https://github.com/epfl-lasa/state-transformers)                |
| [bimanual-dynamical-system](https://github.com/epfl-lasa/bimanual-dynamical-system) |
| [fast-gmm](https://github.com/epfl-lasa/fast-gmm.git)|

---

###Modular Architecture Description:
![alt tag](https://github.com/epfl-lasa/bimanual-task-motion-planning/blob/master/img/lasa-bimanual-architecture.png)

---
###*Real-time* Execution of Bi-manual reaching motions:
#####Stream Robot data (joint states, Pose, FT, Stiff)
```
$ rosrun kuka_fri_bridge run_lwr.sh right
```
```
$ rosrun kuka_fri_bridge run_lwr.sh left
```
follow instructions on [kuka_bridge](https://github.com/nbfigueroa/kuka_interface_packages) to set control mode for each robot. (joint impedance control, control = 1)

#####Visualization and sensor bringup (ft sensors)
```
$ roslaunch kuka_lwr_bringup bimanual2_realtime.launch ft_sensors:=true
```
#####Low-level Controllers
Cartesian-to-Joint/Joint-to-Cart Estimation
```
$ roslaunch state_transformers bimanual_joint_ctrls_real.launch 
```

#####Cartesian Trajectory Generator
A bimanual action server, containing different types of control methods for bimanual actions, currently: 
- Independent CDS for each arm (no coupling) 
- Virtual Object Dynamical System (spatial and temporal coupling)

```
$ roslaunch bimanual_motion_planner bimanual_action_server.launch
```

##### Action Planning  

To run a test with two CDS models independently for each arm:
```
$ rosrun bimanual_action_planners uncoupled_test.py
```

To run a test with the Virtual Object Dynamical System:
```
$ rosrun bimanual_action_planners virtual_object_test.py
```

To run the ***peeling task demo***:
```
$ rosrun bimanual_action_planners peeling_demo.py
```

You will need the whole *Perception Module* for detecting the Zucchini and computing its observable features. 
- Follow the instructions in [kinect-process-scene](https://github.com/nbfigueroa/kinect-process-scene)

---
###*Simulation* of a Bi-manual reaching motions:
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
$ roslaunch bimanual_motion_planner bimanual_action_server.launch simulation:=true
```

##### Action Planning  

To run a test with two CDS models independently for each arm:
```
$ rosrun bimanual_action_planners uncoupled_test.py
```

To run a test with the Virtual Object Dynamical System:
```
$ rosrun bimanual_action_planners virtual_object_test.py
```

---
###Demonstration Recording for Bimanual Tasks:

To record/replay demonstrations you must install these packages:

| Dependencies  |
| ------------- |
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

#####Extract topics to Mat file
Use  [my-matlab-rosbag](https://github.com/nbfigueroa/my_matlab_rosbag)

---


