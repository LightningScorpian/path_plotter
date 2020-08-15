The path plotter_node has to be launched separately.

It reads the transformations from the tf node to and plots positin of the robots base_link frame with respect to the map frame.
These values can be changed from within the path_plotter_node.cpp file in the Plot() function.

The output is a nav_msgs/Path Message on a private topic followoed_path. This path can be visualised by rviz. This path resets when the robot has reached the final goal pointe (when mission executive reports waiting for path);

To build
catkin build


To run
rosrun path_plotter path_plotter_node