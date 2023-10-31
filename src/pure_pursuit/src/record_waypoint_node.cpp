#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using std::placeholders::_1;
using namespace std;

class RecordWaypoint : public rclcpp::Node
{
    // Implement waypoint recorder
    private:
        // file path string
        std::string waypoint_file_path;

        // output file handler
        ofstream waypoint_file;

        // subscriber
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscription_;

        void record_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr point_msg) {

            // callback function for clicked point messages.
            // Write it into the waypoint file

            double x = point_msg->point.x;
            double y = point_msg->point.y;
            double z = point_msg->point.z;

            waypoint_file << x << ' ' << y << ' ' << z << '\n';
            RCLCPP_INFO(this->get_logger(), "waypoint recorded");
        }

    public:

    RecordWaypoint() : Node("record_waypoint_node")
    {
        // declare parameters
        this->declare_parameter("waypoint_file_path", "/sim_ws/waypoints.txt");

        // read parameters
        waypoint_file_path = (this->get_parameter("waypoint_file_path")).as_string();

        // open the file
        waypoint_file.open(waypoint_file_path);
        RCLCPP_INFO(this->get_logger(), "waypoint file opened");

        // subscribe to the clicked point topic
        point_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>("clicked_point", 10,
                                std::bind(&RecordWaypoint::record_callback, this, _1));

    }

    ~RecordWaypoint()
    {
        // close the file
        waypoint_file.close();
        RCLCPP_INFO(this->get_logger(), "waypoint file closed");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RecordWaypoint>());
    rclcpp::shutdown();
    return 0;
}