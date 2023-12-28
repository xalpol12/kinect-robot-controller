## Kinect Robot Controller
> [!NOTE]
> ROS2 package for controlling a robot using image processing with OpenCV and Kinect v1 depth sensor 

## Demo

Full video can be viewed on [youtube]()

Package processes received depth image in order to determine the nearest contour with the largest area between some distance in space (it's the depth section of about 0.3 to 0.6 meter from the sensor - anything outside of this section will be discarded from the potential contour check). Then the position of detected contour's center is calculated and displayed on the final image. Published robot control command is determined by the position of the calculated center in relation to the whole window. 

## Installation 

### Dependencies
This project is a ROS2 package, so naturally ROS2 is required. Project was created using ROS2 Iron, installation guide can be found [here](https://docs.ros.org/en/iron/Installation.html). 

Implemented node also uses `TurtleSim` for simulation. 

```shell
sudo apt update 
sudo apt install ros-iron-turtlesim
```

> [!TIP]
> Don't forget to source your workspace using `source install/setup.bash` (or global source - depends on your config) in order to use ROS commands and have access to installed packages. 
> If something doesn't work, first try sourcing again.

### 1. Install `kinect_ros2`

The `kinect-robot-controller` package depends on `kinect_ros2`. Please refer to 
[kinect_ros2](https://github.com/fadlio/kinect_ros2) installation guide for detailed instructions.

### 2. Copy the repository

Copy the repository to your workspace source folder

```shell
cd ~/ws/src
git clone https://github.com/xalpol12/kinect-robot-controller
```

### 3. Install missing ROS dependencies

```shell
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build your workspace

```shell
cd ~/ws
colcon build
```

### Launch 
Having build the workspace, you can now run individual nodes provided by this package - available nodes are described below. You can also launch all of them at once.

Make sure that the Kinect sensor is connected and system recognises it, then run these commands:
```shell
cd ~/ws
source install/setup.bash
ros2 launch src/kinect-robot-controller/launch/kinect-robot-controller.yml
```

You should see TurtleSim window first, then RBG Kinect image window should start.

Define desired velocity and rotation speed by using sliders and place your hand in one of the defined sectors to move a turtle.

## Detailed description

### `rqt_graph` node view

Whole package integration with Kinect and TurtleSim is illustrated by the given graph:

![rqt ROS graph](/.docs/rosgraph.png)

### Node descriptions

`rgbd_publisher`

Subscribes to `/depth/image_raw` and `/image_raw` topics of `kinect_ros2` package. Normalizes the image and packages it into one variable that is then published on `/rgbd_image` topic.

`image_processor`

Uses contour detection and thresholding operation to create a working area between 0.3 and 0.6 meter. Discards other contours that are not contained in the defined working area. Publishes the largest detected contour in the rectangle form (x, y, w, h) using `rect_pos` topic.

`kinect_robot_controller`

Subscribes the `rect_pos` topic in order to read the rectangle position from `image_processor` node. Defines 9 image sectors that can be mapped to publish certain robot movement messages. 
In a given example, using TurtleSim:
- upper and bottom middle sector (2, 8) - moves forward and backwards by a given linear speed
- left and right middle (4, 6) - rotates left and right by a given rotational speed
- corners - (1, 3, 7, 9) - combines rotational and linear movement
- middle sector (5) - no action 

### Further development 

`kinect_robot_controller` provides one way to implement the detected contour, but one might wish to create different node in order to control a robot, providing different logic. ROS package architecture promotes modularity, adding new nodes for another robots shouldn't cause any problems.

## Used technology
Package was developed using the following technologies: 
- Hardware: Kinect Model 1414 with external USB adapter
- Software: 
  - Linux Ubuntu 22.04 
  - ROS2 Iron
  - Python 3.10

## Contributors
Project was created in cooperation with @przWaw, who also proposed the idea of developing a package for ROS using Kinect depth sensor. Just for fun. Because why not.