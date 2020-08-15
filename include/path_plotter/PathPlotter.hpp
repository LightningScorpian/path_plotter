#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <actionlib_msgs/GoalStatusArray.h>


class PathPlotter {
    private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    
    std::vector <geometry_msgs::PoseStamped> pathVec;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    bool goalActive;
    

    public:
    PathPlotter(const ros::NodeHandle& nh, const std::string& goalStatusArrayTopic);
    ~PathPlotter();

    bool isActive();
    void updateState(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

    void plot(const std::string& sourceFrame, const std::string& destFrame);
    void clearPathVec();
    

};