# urman
ROS2 packages for the UR manipulator with a Robotiq gripper


## Getting Started
***Required setup : Ubuntu 22.04 LTS***

1.  Install `ros2` packages. The current development is based of `ros2 humble`. Installation steps are described [here](https://docs.ros.org/en/humble/Installation.html).
2. Source your `ros2` environment:
    ```shell
    source /opt/ros/humble/setup.bash
    ```
    **NOTE**: The ros2 environment needs to be sources in every used terminal. If only one distribution of ros2 is used, it can be added to the `~/.bashrc` file.
3. Install `colcon` and its extensions :
    ```shell
    sudo apt install python3-colcon-common-extensions
     ```
3. Create a new ros2 workspace:
    ```shell
    mkdir ~/ros2_ws/src
    ```
4. Pull relevant packages, install dependencies, compile, and source the workspace by using:
    ```shell
    cd ~/ros2_ws
    git clone https://github.com/maberobotics/ilab_ur_robot.git src/ilab_ur_robot
    vcs import src < src/ilab_ur_robot.repos
    rosdep install --ignore-src --from-paths . -y -r
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    source install/setup.bash
    ```
**NOTE:** The `ilab_ur_robot.repos` file contains links to ros2 packages that need to be source-built to use their newest features.

## Usage
The package comes with a general purpose launch file, which can be run using 
```shell
$ ros2 launch ilab_ur_bringup ilab_ur.launch.py
```
Several launch arguments can be used to adapt the launch. These arguments can be shown by running 
```shell
ros2 launch ilab_ur_bringup ilab_ur.launch.py --show-args
```

For example, to start the UR real robot with Moveit2 planning and Rviz2, run
```shell
ros2 launch ilab_ur_bringup ilab_ur.launch.py use_fake_hardware:=false use_planning:=true start_rviz:=true robot_ip:=172.16.0.3
```