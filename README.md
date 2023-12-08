# ros2-wheelchair
> A ROS2 Humble and Gazebo Ignition Fortress implementation of a voice-controlled wheelchair<br>
> The implementation combines Gazebo's SDF (in this repo it's in xml for easier development in IDE), ROS2 CMake Package, and ROS2 python package

## Features
1. Voice-Controlled Locomotion
2. Contact-Sensor Stopping (the wheelchair will stop when it detects contact with the wall)
3. Keyboard Teleoperation
4. Too Fast Alert (the wheelchair will try to slowdown when it's too fast and sound an alert)

## Voice Commands
1. Forward
2. Backward
3. Stop
4. Rotate Left x / Rotate Counterclockwise x
5. Rotate Right x / Rotate Clockwise x
6. Cancel (cancel the rotation)

## Keyboard Teleoperations
1. W for forward
2. S for backward
3. A for counterclockwise rotation
4. D for clockwise rotation
5. Q for stopping the wheelchair

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
6. Run the gazebo simulation first `gazebo ign world/gazebo.xml` (if you run it from the repository's root)
7. Run the launch file for this repo `ros2 launch wheelchair wheelchair_launch.yaml` (run it from the repository's root for the alert audio file)
