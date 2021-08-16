![](https://i.imgur.com/ipB5pAy.jpg)
# Visual Servoing of a Moving Target by an Unmanned Aerial
Vehicle
## Introduction
To track moving targets undergoing unknown translational and rotational motions, a tracking controller is developed for unmanned aerial vehicles (UAVs). 
The main challenges are to control both the relative position and orientation between the target and the UAV to within desire values, 
and to guarantee that the generated control input to the UAV is feasible (i.e., below its motion capability). 
Moreover, the UAV is controlled to ensure that the target always remains within the field of view of the onboard camera. 
These control objectives were achieved by developing a nonlinear-model predictive controller, 
in which the future motion of the target is predicted by quadratic programming (QP). 
Since constraints of the feature vector and the control input are considered when solving the optimal control problem, 
the control inputs can be bounded and target can remain inside the image. 

## Contents
* [Installation](https://github.com/Networked-Control-Robotics-Lab/uav_nmpc_tracking_task/blob/master/README.md#installation)
    * [Environment Setup](https://github.com/Networked-Control-Robotics-Lab/uav_nmpc_tracking_task/blob/master/README.md#environment-setup)
    * [YOLO](https://github.com/Networked-Control-Robotics-Lab/uav_nmpc_tracking_task/blob/master/README.md#yolo)
    * [Tracking Controller](https://github.com/Networked-Control-Robotics-Lab/uav_nmpc_tracking_task/blob/master/README.md#tracking-controller)
* [Implementation](https://github.com/Networked-Control-Robotics-Lab/uav_nmpc_tracking_task/blob/master/README.md#implementation)

## Installation
### Environment Setup
* [Install ROS](http://wiki.ros.org/ROS/Installation)
* [Install Gazebo](https://dev.px4.io/v1.9.0/en/simulation/ros_interface.html) 

Gazebo is often used with ROS, a toolkit/offboard API for automating vehicle control. 
* [Import Car model in Gazebo](https://github.com/osrf/car_demo)

The car is assumed to be the moving target to track. The moving target can be any detectable obeject pre-defined by the users. 
### YOLO
* [Install YOLO](https://github.com/n8886919/YOLO#Licence-Plate-Detection)

YOLO is utilized to obtain the image features and the relative angle from the bounding box of the moving target.
![](https://i.imgur.com/hAZaUgz.png)
### Tracking Controller
* Create an empty folder in catkin_ws/src
* [Install Tracking Controller](https://github.com/Networked-Control-Robotics-Lab/uav_nmpc_tracking_task)
 ```
cd ~/catkin_ws/src/uav_nmpc_tracking_task
```

(The default package name is "uav_nmpc_tracking_task". The name can be self-defined by yourself)

```
git clone https://github.com/Networked-Control-Robotics-Lab/uav_nmpc_tracking_task.git
```
```
cd ~/catkin_ws
```

```
catkin_make
```
## Implementation
* Run the model of the UAVs and the car in Gazebo
```
roslaunch uav_nmpc_tracking_gazebo uav_nmpc_tracking_model.launch
```
* Run YOLO
```
cd ~/your_yolo_package/YOLO/car 
```
```
py video_node.py uav1_v11 --topic /iris_1/camera_red_iris/image_raw
```
* Run angle filter
```
roscd  uav_nmpc_tracking_control/scrips/
```
```
python movingavg_filt.py iris_1
```
* Run UAV core controller (with manual flight controller and ukf function)
```
roslaunch uav_nmpc_tracking_control uav1_core.launch
```
While UAV takes off, and everything sets up totally
Click buttom 2 - activate tracking controller
manual flight contoller - w:moving in +x a:moving in +y s:landing l:disarming 
* Run QP (quadratic programming)
```
roscd  uav_nmpc_tracking_control/scrips/
```
```
python target_qp.py iris_1 -r 80 -l 600 -w 0.1
```
(-r : loop rate, -l : window lengh, -w : regulator weight)
* Run NMPC (nonlinear model predictive controller)
```
roscd  uav_nmpc_tracking_control/scrips/
```
```
python nmpc.py iris_1
```


To see more researches, please check our website [NCRL](http://ncrl.nctu.edu.tw/).
