# Robot Navigation Using ArUco Markers

This project implements an autonomous robot navigation system using ArUco markers in a simulated ROS 2 environment. The robot uses a camera to detect ArUco markers placed strategically in the environment, which serve as navigation waypoints. The robot follows a sequence of actions (move forward, turn clockwise, turn counterclockwise) based on the markers it detects, effectively navigating from a starting point to a destination by following the marker trail. This approach demonstrates a practical application of computer vision in robotics, specifically for indoor navigation where GPS might not be available or reliable.

## Demo:

- Demo Video: [Watch on YouTube](https://youtu.be/32QoxUDd72c)

## Project Team:

- [Rajan Khade](https://github.com/rajan-31)
- [Aditya AV](https://github.com/ADITYAAV80/)
- [Pralay D. Saw](https://github.com/Pralay19)

## Professor in Charge:
Prof. Sachit Rao


## Technologies Used:

- **ROS 2** (Robot Operating System) - Version: Jazzy
- **Gazebo** - Harmonic
- **OpenCV** - Version: 4.6
- **Python** - Version: 3.12
- **ArUco Library** - Part of OpenCV

## File Structure:

The project consists of several key files, each responsible for specific functionality:

### Robot Description

- **my_robot.urdf.xacro**: Main robot description file that includes other components
- **mobile_base.xacro**: Defines the differential drive robot with two wheels
- **camera.xacro**: Defines the camera link and joints
- **mobile_base_gazebo.xacro**: Contains Gazebo-specific configurations for the robot
- **common_properties.xacro**: Defines common properties like materials and constants

### Navigation System

- **navigation_03.py**: The main navigation node that:
  - Processes camera images
  - Detects ArUco markers
  - Makes navigation decisions
  - Controls robot movement

- **process_camera_images.py**: Handles image processing pipeline:
  - Converts ROS image messages to OpenCV format
  - Detects ArUco markers
  - Estimates marker poses
  - Visualizes results for debugging

### Simulation Environment

- **my_world.sdf**: Defines the Gazebo simulation world with walls, floor, lighting, and ArUco marker placements

## Navigation Algorithm:

The navigation algorithm works as follows:

1. **Initialization**:
   - Define marker dictionary and detection parameters
   - Set up the navigation plan (marker IDs and corresponding actions)
   - Initialize the target marker ID and action

2. **Image Processing**:
   - Capture camera image
   - Convert to grayscale
   - Detect ArUco markers using `aruco.detectMarkers()`
   - Estimate marker poses using `aruco.estimatePoseSingleMarkers()`

3. **Navigation Logic**:
   - If the target marker is detected:
     - Calculate distance to marker and its center position in the image
     - If current action is "turn_cw" or "turn_ccw":
       - If marker is centered in image (x position between 310-330), change action to "move"
     - If current action is "move":
       - If distance to marker is small enough (≤ 4 units), update target marker ID and action based on navigation plan

4. **Robot Control**:
   - Set robot velocity based on current action:
     - "move": Set linear velocity forward
     - "turn_cw": Set angular velocity clockwise
     - "turn_ccw": Set angular velocity counter-clockwise
     - "idle": Stop all movement
   - Publish velocity command to the robot

## Key ROS Components:

- **Nodes**:
  - `navigation_03`: Processes camera images and controls robot movement
  - `ros_gz_bridge`: Bridges communication between ROS 2 and Gazebo
  - `robot_state_publisher`: Publishes robot state to the TF tree

- **Topics**:
  - `/camera/image`: Camera image data
  - `/camera/camera_info`: Camera calibration information
  - `/cmd_vel`: Robot velocity commands
  - `/joint_states`: Robot joint state information

## Running the Project:

To run the project, follow these steps:

1. Make sure ROS 2 Jazzy and Gazebo Harmonic are installed
2. In terminal go to your ROS 2 workspace
3. Activate your workspace (source `setup.bash`)
4. Build the project using `colcon build` OR `colcon build --packages-select my_robot_description my_test_pkg my_robot_bringup --symlink-install`
5. Source the workspace: `. install/setup.bash`
6. Launch the simulation: `ros2 launch my_robot_bringup my_robot_gazebo.launch.py`
7. Run the navigation node: `ros2 run my_robot_navigation navigation_03`


