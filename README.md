# ECSE 373 Laboratory #5

## Set up

Important: Once downloaded, the folder containing these items should be renamed from the repository name to ariac_entry (this is a logistical error, and is too late to be changed).

This package relies on the ARIAC 2019 environment which you can read more about at (https://bitbucket.org/osrf/ariac/wiki/2019/Home). This version of the package makes use of ROS Noetic.

For this project two separate workspaces are used. A third will also be installed, but portions will be cloned where necessary.

```
# Create a workspace for the simulation environment
mkdir ~/ariac_ws/src
cd ~/ariac_ws/src

# Clone the necessary repository for simulation
git clone https://github.com/cwru-eecs-373/cwru_ariac_2019.git

# Install any missing dependencies for this repository
rosdep install --from-paths . --ignore-src -r -y

# Build the catkin ws for this environment
cd ../
catkin_make

# Install the simulator environment
sudo -- /bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install"

# Make a workspace to clone the ARIAC node.
mkdir ~/ecse_373_ariac_ws/src
cd ~/ecse_373_ariac_ws/src

# Clone the GIT repository for this laboratory
git clone https://github.com/cwru-eecs-373/ecse_373_ariac.git

# Install any missing dependencies for this environemnt
rosdep install --from-paths ecse_373_ariac --ignore-src -r -y

# Add it to your ROS environment
cd ../
catkin_make

# Create a workspace ot house the components specific to this final project
mkdir ~/ariac_project_ws/src

# Copy necessary dependencies to current project workspace
cp -R ~/ecse_373_ariac_ws/src/ecse_373_ariac ~/ariac_project_ws/src/
cp -R ~/ecse_373_ariac_ws/src/ur_kinematics ~/ariac_project_ws/src/
# This can also be done through system platform by copying ecse_373_ariac and ur_kinematics folders to ariac_project_ws/src

# Build the completed workspace for the current project
source devel/setup.bash
catkin_make

```
After this there should be 3 workspaces, two which are relevant. Ariac_ws wich holds the simulation information, ecse_373_ariac which contains necessary packages (ur_kinematics and ecse_373_ariac which can be copied to the final workspace), and ariac_project_ws, which holds the user-written code for handling the simulation.

## Launch

`roslaunch ariac_entry entry.launch`

Wait for the gazebo window to open. Once it is fully loaded, hit the play button at the bottom of the screen to begin.

## Package Objectives

The purpose of this package is to process orders in the ARIAC simulation by:

- Start the competition
	- A strong error message should accompany failed calls to the
	  start_competition service
	- A less strong message should indicate if a successful call to the
	  start_competition service is unsuccessful starting the service and why
- Subscribe to the Orders topic
- Use the material_location service to find the bin(s) that has a part of the type
  required by the first product in the first shipment of the first order.
- Subscribe to all logical_cameras and thoughtfully store the information
- Print to the screen a ROS_WARN[_STREAM] message once, where the x, y, and z pose
  of a part of the correct type in a location identified by the material_location service
  (and the bin) when an order is received. Provide the location of the part in the reference
  frame of the logical_camera, and in the reference frame of the robot
  arm1_base_link.

