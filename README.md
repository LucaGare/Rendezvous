# Rendezvous
ROS repository for the performing of the cooperative autonomous landing of a PX4 Vision UAV on top of a moving Nexus robot with a landing platform mounted on top. The PX4 Vision and the Nexus robot rely on mocap (motion capture) system for absolute position fix.

## Building
To build the package, clone the current repository in your catkin workspace and build it.
```
cd catkin_ws/src
git clone https://github.com/LucaGare/Rendezvous.git
```
Build your workspace with either *catkin_make* or *catkin build*
```
cd ...
catkin_make
```

If necessary, rebuild the shared libraries "MPC.so", "MPC_UGV.so", "rk4.so" and "rk4_UGV.so" in the folder "src/SharedLibs" from the code "MPC.c", "MPC_UGV.c", "rk4.c" and "rk4_UGV.c" in the same folder.

## Usage
**SIMULATION**
* Launch the following file:
```
roslaunch Rendezvous Rendezvous.launch run_simulation:=true
```

**USAGE ON THE REAL SYSTEM**
* Make sure to use battery level alarms on the UGV battery.
* Turn on the UAV's RC controller. 
* Turn on the mocap system and be sure that the model markers are correctly registered.
* Start on the UAV and UGV's own companion computers the interface with the low level controllers. To do so, start on the vehicles' companion computers the following files, respectively:
```
roslaunch Rendezvous PX4Vision_UpCore.launch run_mocap:=true
```
(if it's not needed to run the mocap node), the `run_mocap` argument can be dropped (false by default).
```
roslaunch Rendezvous bringup_nexus5.launch run_mocap:=true
```
* Start the recording node before starting the controllers so that all the relevant messages are recorded (which may not be true in the case the recording node is started contextually to the controllers). To do so launch the file:
```
roslaunch Rendezvous Record_rendezvous.launch
```
* Manually reach the UAV's starting position for the maneuver. 
* Start the controllers in order to perform the maneuver launching the following file:
```
roslaunch Rendezvous Rendezvous.launch
```
