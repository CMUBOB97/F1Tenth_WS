#include "rrt.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRT_ROS_Node>());
    rclcpp::shutdown();
    return 0;
}