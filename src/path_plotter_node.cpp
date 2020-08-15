#include <PathPlotter.hpp>



int main(int argc, char ** argv) {
    ros::init(argc, argv, "path_plotter_node");
    ros::NodeHandle nh("~");
    PathPlotter plotter(nh, "/controller_executive/exe_path/status");
    ROS_INFO_STREAM("Initialised plotter");
    ros::Rate rate(2);
    while (ros::ok()) {
        ROS_INFO_STREAM("Awaiting path");
        while (plotter.isActive() && nh.ok()) {
            plotter.plot("map", "base_link");
            ros::spinOnce();
            rate.sleep();
        }
        plotter.clearPathVec();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}




