# My_beginner_Tutorials

## Overview
This Project contains a beginners tutorial of ROS2 Humble, specifically publishing and subscribibg to a topic.

## Author

Keyur Borad (keyurborad5@gmail.com)

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- C++17 compatible compiler
- colcon build tool
- clang-format
- cpp-lint
- clang-tidy

## Installation
1. Create a colon workspace
```bash
# After installing the ROS2 Humble Open terminal and source the ROS2 underlay
source /opt/ros/humble/setup.bash
# Create new directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

```
2. Clone the repository 
```bash
#Clone the repository
git clone https://github.com/keyurborad5/my_beginner_tutorials.git
```

3. Check for missing dependencies and installing it
```bash
#go to ros2_ws directory
cd ..
rosdep install -i --from-path src --rosdistro humble -y

```
4. Build the workspace with colcon bubild
```bash
colcon build
```

5. Source Overlay
```bash
source install/setup.bash
```

## Running the Package Nodes
1. Run the Publisher Node of the beginner_tutorials Package
```bash
ros2 run beginner_tutorials talker
```
2. Open another terminal and run the subcriber node
```bash
#source underlay
source /opt/ros/humble/setup.bash
# Got to the workspace directory
cd ~/ros2_ws
#source the overlay
source install/setup.bash
#Run the Subscriber node
ros2 run beginner_tutorials listener
```

## Clang-Formating
```bash
cd ~/ros2_ws
#Clang-format 
clang-format -i --style=Google $(find . -name *.cpp -o -name *.hpp | grep -v "/build/")

```
## Cpp-Lint
```bash
# go to your ros2 workspace directory
cd ~/ros2_ws
#Cpp Lint
cpplint --filter=-legal/copyright,-build/c++11,+build/c++17,-build/namespaces,-build/include_order $(find . -name *.cpp | grep -v "/build/")

```
## Clang-tidy
```bash
cd ~/ros2_ws
# Build the workspace again with the camake args to generate compile_commands.jason file for Clang-tidy to work
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
#Clang-tidy command
clang-tidy -p build/beginner_tutorials --extra-arg=-std=c++17 src/beginner_tutorials/src/*.cpp
```

## Acknowledgement

- Open Source Robotics Foundation, Inc.
- ROS2 Community

