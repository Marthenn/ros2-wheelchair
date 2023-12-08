# ros2-wheelchair
> A ROS2 Humble and Gazebo Ignition Fortress implementation of a voice-controlled wheelchair<br>
> The implementation combines Gazebo's SDF (in this repo it's in xml for easier development in IDE), ROS2 CMake Package, and ROS2 python package

## Requirements
1. ROS2 Humble
2. Gazebo Ignition Fortress
3. Python 3
4. Python [SpeechRecognition Library](https://pypi.org/project/SpeechRecognition/)
5. Python [PlaySound Library](https://pypi.org/project/playsound/)

## How to Run
1. Clone this repo and the [interfaces repo](https://github.com/Marthenn/ros2-wheelchair-interfaces)
2. Build the interfaces repo first
3. Source the interfaces build
4. Build this repo
5. Source this repo
6. Run the gazebo simulation first `gazebo ign world/gazebo.xml`
7. Run the launch file for this repo `ros2 launch wheelchair wheelchair_launch.yaml`
