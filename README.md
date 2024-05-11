# Landing on an AruCo marker

This is a work in progress and proof of concept of an off-board control that lands an UAV using PX4 on an AruCo marker.

It uses [Precision Land flight mode](https://docs.px4.io/main/en/advanced_features/precland.html#precision-landing-flight-mode) in combination with AruCo marker detection on bottom camera pictures.

A demo is available at:

[sim.webm](https://github.com/radekholy24/aruco-landing/assets/7268060/f0a37df2-9267-46be-a9f4-075d32091481)

The sources of inspiration are mostly:

* https://youtu.be/iezU2PR0hBk
* https://github.com/Kenil16/master_project

A lot of work need to be done before running in production!

## Dependencies

* [Python](https://en.wikipedia.org/w/index.php?title=Python_(programming_language)&oldid=1223123409) 3.8 or compatible
* [Robot Operating System](https://en.wikipedia.org/w/index.php?title=Robot_Operating_System&oldid=1214363848) noetic with [cv_bridge](http://wiki.ros.org/cv_bridge), [geometry_msgs](http://wiki.ros.org/geometry_msgs), [mavros](http://wiki.ros.org/mavros), [mavros_msgs](http://wiki.ros.org/mavros_msgs), [rospy](http://wiki.ros.org/rospy) and [sensor_msgs](http://wiki.ros.org/sensor_msgs) or compatible
* [Pymavlink](https://github.com/ArduPilot/pymavlink/) 2.4 or compatible
* [NumPy](https://en.wikipedia.org/w/index.php?title=NumPy&oldid=1216944842) 1.24 or compatible
* [OpenCV-Python](https://en.wikipedia.org/w/index.php?title=OpenCV&oldid=1208982530) 4.2 or compatible

## Installation

1. install the dependencies above
2. download the `offboard_control` package to your PYTHONPATH

## Run

1. run the `offboard_control` package with `--help` argument to see its usage

## Tutorials

### Gazebo simulator on Linux Mint

Install ROS:

```shell
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install ros-noetic-desktop ros-noetic-mavros
```

Install GeographicLib datasets:

```shell
# Extracted from https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo geographiclib-get-geoids egm96-5
```

Install OpenCV-Python:

```shell
sudo apt install python3-opencv
```

Create a workspace and a Python virtual environment:

```shell
mkdir precland
python3 -m venv --system-site-packages precland/venv
```

Install PX4:

```shell
git clone --branch=v1.14.2 --recursive --depth=1 https://github.com/PX4/PX4-Autopilot.git precland/PX4-Autopilot
# Extracted from Tools/setup/ubuntu.sh
sudo apt-get --no-install-recommends install libjson-perl cppcheck libxml2-dev shellcheck ninja-build lcov astyle
sudo apt-get --no-install-recommends install libgstreamer-plugins-base1.0-dev
. precland/venv/bin/activate
pip install -r precland/PX4-Autopilot/Tools/setup/requirements.txt
pushd precland/PX4-Autopilot && DONT_RUN=1 make px4_sitl gazebo-classic; popd
deactivate
```

Download a world:

```shell
curl --header "X-GitHub-Api-Version: 2022-11-28" --header "Accept: application/vnd.github+json" --location   https://api.github.com/repos/Kenil16/master_project/tarball/b032d57b9a47796a2e81caebd2f359d79de1d846 | tar --gunzip --extract --directory=precland Kenil16-master_project-b032d57/software/ros_workspace/PX4-software/worlds/optitrack_big_board_onepattern.world Kenil16-master_project-b032d57/software/ros_workspace/PX4-software/models
```

Install the off-board control:

```shell
curl --header "X-GitHub-Api-Version: 2022-11-28" --header "Accept: application/vnd.github+json" --location   https://api.github.com/repos/radekholy24/aruco-landing/tarball/v0.1.0-alpha | tar --gunzip --extract --directory=precland
mv --verbose precland/radekholy24-aruco-landing-* precland/radekholy24-aruco-landing
```

Run the simulation:

```shell
cd precland
. venv/bin/activate
. /opt/ros/noetic/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(pwd)/Kenil16-master_project-b032d57/software/ros_workspace/PX4-software/models:$(pwd)/radekholy24-aruco-landing/sim
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/PX4-Autopilot:$(pwd)/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/radekholy24-aruco-landing
roslaunch $(pwd)/radekholy24-aruco-landing/sim/sim.launch \
    "world:=$(pwd)/Kenil16-master_project-b032d57/software/ros_workspace/PX4-software/worlds/optitrack_big_board_onepattern.world" \
    "vehicle:=gazebo-classic_iris" \
    "vehicle_sdf:=$(pwd)/radekholy24-aruco-landing/sim/gazebo-classic_iris_camera/gazebo-classic_iris_camera.sdf" \
    "vehicle_x:=-3" \
    "vehicle_y:=0" \
    "vehicle_z:=0" \
    "vehicle_R:=0" \
    "vehicle_P:=0" \
    "vehicle_Y:=0" \
    "offboard_control_camera_matrix:=623.680552,0,360,0,623.680552,240,0,0,1" \
    "offboard_control_distortion_coefficients:=0,0,0,0" \
    "offboard_control_search_altitude:=5.5" \
    "offboard_control_bottom_camera_node_name:=mono_cam_bottom" \
    "offboard_control_land_on_aruco_rate:=20" \
    "offboard_control_aruco_markers:=5X5_1000:537:0.2 5X5_1000:467:0.2 5X5_1000:537:0.2"
```

It is supposed to take off, switch to the Precision Land flight mode, find the given marker and land on it. Then do the same for the other given marker. And then again for the first marker.
