The following was tested on ROS Noetic and python 3.8.
Python packages are required.
torch                           1.13.1
torchaudio                      0.13.1
torchvision                     0.14.1
open3d                          0.13.0
numpy                           1.24.3
scikit-spatial                  7.0.0

Installation guide:
1. Install turtelbot3 packages
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/

2. Install packages for open manipulator and turtlebot3
https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#bringup

3. Clone the project to your catkin/src:
git clone 
final src file should contain the following files:
hls_lfcd_lds_driver, open_manipulator_dependencies, point_cloud_edge_and_corner_detection
turtlebot3_manipulation, turtlebot3_manipulation_simulations, turtlebot3_simulations




Simulated robot:
roslaunch point_cloud_edge_and_corner_detection sim_robot.launch 


Real hardware:
roslaunch turtlebot3_bringup turtlebot3_robot.launch
roslaunch point_cloud_edge_and_corner_detection bringup.launch
rosrun point_cloud_edge_and_corner_detection pointcloud.py 
