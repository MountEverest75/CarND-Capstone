# System Integration (Building ROS Nodes)
## Self Driving Car Nano-degree

### Our Team - Carla Chargers

| Name | Contribution | Email |
| --- | --- | --- |
| Ravi Kiran Chebolu | Team Lead and Waypoint Updater development | ravikiran_prof@yahoo.com or ravikiranmailid@gmail.com |
| Qi Peng | Traffic Light Detection development| pengqi193@gmail.com |
| Inhan Song | Control Module development| cyrano5614@gmail.com |
| Yasuyaki Kataoka | Environment Setup, Reviews and Testing | yk1002jp@gmail.com |


### Introduction
The purpose of this project is to build ROS nodes implementing core functionalities of an autonomous vehicle system, test the code in simulator and run on Carla. The core ROS nodes that need to be built are listed below:  
* Traffic light detection
* Control
* Waypoint following

The system architecture with ROS nodes and the topics used to establish communication between is given below:

![](./imgs/final-project-ros-graph-v2.png)

### Pre-requisites
The following python component is required to avoid some errors we may encounter related traffic light detection waypoint inside docker container:

```
pip install pillow --update
```

### Development

#### Waypoint Updater
For waypoint updates, the logic from path planning project has been re-purposed and written in Python file waypoint_updater.py.

#### Traffic Light detection
Two approaches have been tried.
* Approach 1: A model has been trained using the sample images for traffic lights and used in detection logic to classify images as red light waypoints
* Approach 2: Using Computer Vision library to detect traffic light waypoint and identify it as red or green.

After testing extensively we chose to implement Approach 2 in the final master branch to publish details in /image_color topic. The source code for Approach 1 is available in "tested" branch.

The changes to classify traffic light are made in tl_classifier.py. The changes to detect traffic light in waypoints can be found in tl_detector.py.

#### Control module
The car simulator receives commands from the control module to steer, brake and accelerate the car. The logic can be found in dbw_node.py file.

### Testing and Results
We have observed latency with waypoint updates while using simulator in Mac with ROS VM provided by Udacity. We have observed better performance of simulator while running on machines with Native ubuntu installations and GPU processors. The following [video](https://youtu.be/F1mJKhlT76I) clip shows successful execution of test in simulator.

### Conclusion
The architecture using publish/subscribe model between ROS nodes has been found to be robust and non blocking. One of the improvements that can be done would be to train the traffic light detection and use the model to classify. Another amendment would be to refactor the code for execution in a church lot which has been recently added in the simulator. Overall the results have been satisfactory. Thanks to the Awesome team work !

----
### Udacity Documentation

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
