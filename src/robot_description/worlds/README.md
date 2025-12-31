# To get Gazebo qithout models working

## Good place for models
git clone https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps
git clone https://github.com/osrf/gazebo_models ~/.gazebo/models

## Example of getting small house working
git clone https://github.com/aws-robotics/aws-robomaker-small-house-world ~/aws_models

Test the models
export GZ_SIM_RESOURCE_PATH=\
~/aws_models/models:\
$GZ_SIM_RESOURCE_PATH

Test world in gazebo
gz sim /home/luke/Projects/capstone/search-support-robots/ros-packages/robot_description/world/small_house.world

Make permanant
echo 'export GZ_SIM_RESOURCE_PATH=~/aws_models/models:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc

## Test inside ros2_ws

Test the models
export GZ_SIM_RESOURCE_PATH=\
~/capstone/search-support-robots/ros2_ws/src/robot_description/models/aws:\
$GZ_SIM_RESOURCE_PATH

Test world in gazebo
gz sim ~/capstone/search-support-robots/ros2_ws/src/robot_description/worlds/small_house.world
