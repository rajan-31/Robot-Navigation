Getting started



- Put src folder in your ROS2 workspace
- In terminal go to your ROS2 workspace
- Activate your workspace (source `setup.bash`)
- `colcon build` OR `colcon build --packages-select my_robot_description my_test_pkg my_robot_bringup --symlink-install`
- Again source `setup.bash`
- `ros2 launch my_robot_bringup my_robot.launch.xml`
- `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- `ros2 run my_test_pkg process_camera_images`

---

Misc

- export GZ_SIM_RESOURCE_PATH=~/.gazebo/models/

Spaw ArUco Marker - Resource Spawner
- top left menu

---

- Figure out depth values
- Generated ArUco marker image - OpenCV
- Create Wall Models - 

---

### Poblem statement

We have a room