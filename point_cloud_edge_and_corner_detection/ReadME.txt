
Run the rosbag file 
roscore
rosbag play -l edge.bag


/camera/depth/color/points /camera/depth/image_rect_raw /camera/color/image_raw

bag_to_pcd
$ rosrun pcl_ros bag_to_pcd <input_file.bag> <topic> <output_directory>


roslaunch realsense2_camera rs_camera.launch filters:=pointcloud ordered_pc:=true clip_distance:=1.5







<param name="move_group/trajectory_execution/allowed_execution_duration_scaling" value="4.0" />
  <param name="move_group/trajectory_execution/execution_duration_monitoring" value="true" />

  <!-- send robot urdf to param server -->
  <param name="robot_description" 
  command="$(find xacro)/xacro --inorder '$(find point_cloud_edge_and_corner_detection)/urdf/turtlebot3_manipulation_robot.urdf.xacro'"/>

  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
    <param name="publish_frequency" type="double" value="50.0" />
  </node>


  
  <include file= "$(find realsense2_camera)/launch/rs_camera.launch">
    <arg name="filters" value="pointcloud" />
    <arg name="clip_distance" value="1.0" />

  </include>


Issue with tf2_geometry_msg message ros as one dependency cannot be imported to ros noetic as it is not supported
Made changes to relocate the end_effector_link from 0.126 to 0.16