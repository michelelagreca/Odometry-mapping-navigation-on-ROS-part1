# Odometry Calculation for Autonomous Shuttle
This README file provides detailed information about the implementation of odometry calculation for an experimental autonomous shuttle, EasyMile EZ10, using ROS (Robot Operating System). The project aims to estimate the shuttle's position and orientation based on wheel speed and steering angle data obtained from a ROS bag file.

## Problem Description
The objective of this project is to calculate the odometry of an autonomous vehicle, specifically the EasyMile EZ10 experimental autonomous shuttle. The odometry estimation is based on wheel speed and steering angle measurements. The shuttle has only front steering and a distance of 2.8 meters between the front and rear wheels. The provided bag file contains data for the speed and steering angle of the shuttle.

## Implementation

### Requirements
- ROS (Robot Operating System) installed
- C++ programming environment for ROS

### ROS Bag File
To use the ROS bag file, follow these steps:
1. Place the bag file in a suitable directory.
2. Open a terminal and navigate to the directory where the bag file is located.
3. Execute the following command to play the bag file:
```
rosbag play --clock -l <bag_file_name>.bag
```
Replace `<bag_file_name>` with the actual name of your bag file.

### Code Explanation
The code provided consists of several components:

#### Odom Node
The main node responsible for odometry calculation is `odom_node`. It subscribes to the `/speed_steer` topic (published by the bag file) to receive the speed and steering angle data and publishes the odometry (as a nav_msgs/Odometry message) and custom odometry messages (as Odom custom message).

#### Custom Message
The custom message type `first_project/Odom` is defined to store the calculated odometry information. It contains the following fields:
- `x`: Float value representing the x-coordinate of the shuttle's position.
- `y`: Float value representing the y-coordinate of the shuttle's position.
- `th`: Float value representing the orientation angle (theta) of the shuttle.
- `timestamp`: String value representing the timestamp of the odometry calculation.

#### Odometry Message
The standard ROS message type `nav_msgs/Odometry` is used to publish the odometry information. It contains the calculated position and orientation of the shuttle.

#### Transform Broadcast
The code utilizes the `tf::TransformBroadcaster` to broadcast the transformation between the `odom` and `base_link` frames. This allows visualization of the shuttle's movement in RViz.

#### Service
The `reset_odom` service is implemented to reset the odometry to zero upon request. It takes no input parameters and returns a boolean value indicating whether the odometry was successfully reset.

### Running the Code
To run the code, follow these steps:
0. We create a ROS folder: then we create src sub folder (where we put the code and the packages), then we go in ROS folder and we build the system with catkin_make (we do again this command to compile new things that are in src)
1. Create a ROS package named `first_project`.
2. Place the provided code files within the package directory.
3. Build the ROS package using the appropriate ROS build commands (e.g., `catkin_make`).
4. Ensure that the bag file is available and its name is correctly specified in the code.
5. Launch Tmux, so that it is possible to have multiple terminal windows at the same time.
6. Launch the project using the provided launch file:
```
roslaunch first_project first_project.launch
```
7. Observe the published odometry and tf frames in RViz, with the fixed frame set to `odom`.
8. It is possible to open another tmux windows and run the service with the command:
```
rosservice call /reset_odom
```

## Results
Upon running the code and launching the project, the odometry of the EasyMile EZ10 shuttle will be calculated based on the speed and steering angle data from the bag file. The calculated odometry information will be published as custom odometry and standard odometry messages. Additionally, the transformation between the `odom` and `base_link` frames will be broadcasted, allowing visualization of the shuttle's movement in RViz.

## Contributing
This project has been implemented with the contribution of Andrea Giuffrida and Sara Zoccheddu.

## License
This project is licensed under the [MIT License](LICENSE).




