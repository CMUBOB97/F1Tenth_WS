/// CHECK: include needed ROS msg type headers and libraries
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers

        // declare parameters
        this->declare_parameter("car_length", 0.3302);
        this->declare_parameter("ttc_threshold", 1.0);
        this->declare_parameter("laser_threshold", 20.0);
        
        // subscriber and callback functions
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10,
                              std::bind(&Safety::scan_callback, this, _1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("ego_racecar/odom", 10,
                              std::bind(&Safety::drive_callback, this, _1));

        // ackermann message publisher
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    }

private:
    double speed = 0.0;
    double angular = 0.0;
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        speed = msg->twist.twist.linear.x;
        angular = msg->twist.twist.angular.z;
        RCLCPP_INFO(this->get_logger(), "speed update: %f", speed);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC
        /// TODO: publish drive/brake message

        // get parameters
        rclcpp::Parameter c_length = this->get_parameter("car_length");
        rclcpp::Parameter ttc_low = this->get_parameter("ttc_threshold");
        rclcpp::Parameter laser_high = this->get_parameter("laser_threshold");

        // readings
        std::vector<float> readings = scan_msg->ranges;
        uint32_t readings_size = readings.size();
        std::vector<float> ttc(readings_size, 0.0);
        double angle_current = scan_msg->angle_min;
        double angle_increment = scan_msg->angle_increment;
        double laser_speed;

        // calculate the com lateral speed
        if (abs(angular) >= 1e-7) {
            double R = speed / angular;

            // need to get how much the wheel steered?
            // delta = steering angle
            // project v onto beta
            // where beta = atan(l_r / (l_f + l_r) * tan(delta))
        }

        for (uint32_t i = 0; i < readings_size; i++) {
            
            // filter out invalid lidar readings
            if (readings[i] > laser_high.as_double()) {
                ttc[i] = -1.0;
            } else {
                // calculate laser beam speed
                laser_speed = speed * cos(angle_current);

                // calculate ttc
                ttc[i] = readings[i] / laser_speed;
            }

            // if ttc is lower than a threshold, stop
            if (ttc[i] > 0 && ttc[i] < ttc_low.as_double()) {
                RCLCPP_INFO(this->get_logger(), "possible collision detected at angle: %f", angle_current);

                // publish
                auto message = ackermann_msgs::msg::AckermannDriveStamped();
			    message.drive.speed = 0.0;
                ackermann_publisher_->publish(message);
            }
            
            // increment angle
            angle_current += angle_increment;
        }
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
