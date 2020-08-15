#include <PathPlotter.hpp>

PathPlotter::PathPlotter(const ros::NodeHandle& nh, const std::string& goalStatusArrayTopic) : 
    tfBuffer {}, 
    tfListener{tfBuffer},
    goalActive {false}
{
    this -> nh = nh;
    ROS_INFO_STREAM("[Path Plotter] : Initialising Path Plotter");
    pathVec.reserve(1000);
    pub = (this -> nh).advertise<nav_msgs::Path>("followed_path", 1000);
    sub = (this -> nh).subscribe(goalStatusArrayTopic, 10000, &PathPlotter::updateState, this);
}

PathPlotter::~PathPlotter() {
    ROS_INFO_STREAM("[Path Plotter] : PathPlotter object out of scope");
}

void PathPlotter::plot(const std::string& source, const std::string& dest) {
    geometry_msgs::TransformStamped tfStamped;
    nav_msgs::Path path;
    try {
        tfStamped = tfBuffer.lookupTransform(source, dest, ros::Time(0));
        ROS_INFO_STREAM("[Path Plotter] : Captured latest point : " << tfStamped.transform.translation.x << ' ' << tfStamped.transform.translation.y);
        geometry_msgs::PoseStamped currPos;

        
        currPos.header = tfStamped.header;
        currPos.pose.position.x = tfStamped.transform.translation.x;
        currPos.pose.position.y = tfStamped.transform.translation.y;
        currPos.pose.position.z = tfStamped.transform.translation.z;

        currPos.pose.orientation.w = tfStamped.transform.rotation.w;
        currPos.pose.orientation.x = tfStamped.transform.rotation.x;
        currPos.pose.orientation.y = tfStamped.transform.rotation.y;
        currPos.pose.orientation.z = tfStamped.transform.rotation.z;
        
        pathVec.push_back(currPos);
        path.poses = pathVec;
        path.header = tfStamped.header;
        path.header.seq = pathVec.size();
        pub.publish(path);


    } catch (tf2::TransformException& te) {
        ROS_WARN_STREAM("[Path Plotter] : Couldn't read transform frame");
        return;
    }

}


void PathPlotter::updateState(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
    //ROS_INFO_STREAM("callback called " << (msg -> status_list).empty());
    if ((msg -> status_list).empty()) {
        ROS_INFO_STREAM("[Path Plotter] : Control no longer active");
        goalActive = false;
    } else {
        ROS_INFO_STREAM("[Path Plotter] : Control active");
        goalActive = true;
    }
    //goalActive = !(msg -> status_list.empty()); 
}

bool PathPlotter::isActive() {
    return goalActive;
}

void PathPlotter::clearPathVec() {
    pathVec.clear();
}