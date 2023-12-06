#include <string>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers

        // declare PID parameters
        this->declare_parameter("kp", 3.0);
        this->declare_parameter("ki", 0.0);
        this->declare_parameter("kd", 0.01);
        this->declare_parameter("dist", 1.0);
        this->declare_parameter("beam_a_start", -1.5708);
        this->declare_parameter("beam_b_start", -1.0472);
        this->declare_parameter("angle_range", 1.2276);
        this->declare_parameter("odom_topic_name", "ego_racecar/odom");
        this->declare_parameter("valid_reading_range", 20.0);
        this->declare_parameter("following_left_wall", false);

        // read odom topic name
        std::string odom_topic = (this->get_parameter("odom_topic_name")).as_string();

        // subscribers
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("opp_scan", 10,
                              std::bind(&WallFollow::scan_callback, this, _1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 10,
                              std::bind(&WallFollow::drive_callback, this, _1));

        // publishers
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("opp_drive", 10);
    }

private:
    // PID CONTROL PARAMS
    // TODO: define kp, ki, kd
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double current_velocity = 0.0;
    double prev_error = 0.0;
    double integral_error[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    uint8_t integral_counter = 0;

    // angle ranges and increments
    double ang_min;
    double ang_max;
    double ang_increment;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    double get_range(std::vector<float>& range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle.
        Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // TODO: implement

        // find the index of the reading corresponding to that angle
        int range_data_length = range_data.size();
        int index = (range_data_length * (angle - ang_min)) / (ang_max - ang_min);
        
        if (index < 0 || index > range_data_length) {
            RCLCPP_ERROR(this->get_logger(), "ERROR! index out of range!");
            return 40.0;
        } else {
            return range_data[index];
        }
    }

    double get_error(std::vector<float>& range_data, double dist, double velocity)
    {
        /*
        Calculates the error to the wall.
        Follow the wall to the left (going counter clockwise in the Levine loop).
        You potentially will need to use get_range().

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall
            velocity: current vehicle velocity - for look ahead

        Returns:
            error: calculated error
        */

        // TODO:implement

        // define where to start searching valid wall readings
        double a_angle = (this->get_parameter("beam_a_start")).as_double();
        double b_angle = (this->get_parameter("beam_b_start")).as_double();

        // define which side of wall is currently being followed
        bool follow_left = (this->get_parameter("following_left_wall")).as_bool();

        // store valid beam a and b readings (40m is beyond the sensor range)
        double a_valid = 40.0;
        double b_valid = 40.0;
        double valid_reading_range = (this->get_parameter("valid_reading_range")).as_double();

        // read angle range parameter (max angle between beam a and b, about 70 degrees)
        static double angle_range = (this->get_parameter("angle_range")).as_double();

        /*
        read laser values at a and b
        note that at least one beam must be orthogonal to vehicle heading
        if following left wall - a must be orthogonal
        if following right wall - b must be orthogonal
        */
        if (follow_left) {

            // the only a reading
            a_valid = get_range(range_data, a_angle);

            // b has a range to look up
            for (; (b_angle > (a_angle - angle_range)) && (b_valid > valid_reading_range); b_angle -= ang_increment) {
                b_valid = get_range(range_data, b_angle);
            }

        } else {

            // the only b reading
            b_valid = get_range(range_data, b_angle);

            // a has a range to look up
            for (; (a_angle < (b_angle + angle_range)) && (a_valid > valid_reading_range); a_angle += ang_increment) {
                a_valid = get_range(range_data, a_angle);
            }

        }

        // use a and b to calculate distance to wall
        double error = 0.0;
        double theta, alpha, dist_actual;

        // double check sensor readings are in the right range
        if (a_valid < valid_reading_range && b_valid < valid_reading_range) {
            
            theta = a_angle - b_angle;
            // left or right wall following - different formula
            if (follow_left) {
                alpha = atan2(a_valid - b_valid * cos(theta), b_valid * sin(theta));
                dist_actual = a_valid * cos(alpha);
                error = dist_actual - velocity * sin(alpha) - dist;
            } else {
                alpha = atan2(a_valid * cos(theta) - b_valid, a_valid * sin(theta));
                dist_actual = b_valid * cos(alpha);
                error = dist - dist_actual - velocity * sin(alpha);
            }
            
        }

        RCLCPP_INFO(this->get_logger(), "error: %f", error);
        return error;
    }

    void pid_control(double current_error, double previous_error, double* past_errors)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            current_error: error calculated at current time stamp
            previous_error: error calculated at last time stamp
            past_errors: array of past 5 calculated errors
            * these terms are used for PID controller

        Returns:
            None
        */

        // TODO: Use kp, ki & kd to implement a PID controller

        // calculate the steering angle using PID
        double error_sum = 0.0;
        for (int i = 0; i < 5; i++) {
            error_sum += past_errors[i];
        }
        
        // read PID parameters
        kp = (this->get_parameter("kp")).as_double();
        ki = (this->get_parameter("ki")).as_double();
        kd = (this->get_parameter("kd")).as_double();
        
        double angle = kp * current_error + ki * error_sum + kd * (current_error - previous_error);

        // cap at +/- pi
        if (angle > 3.14) {
            angle = 3;
        } else if (angle < -3.14) {
            angle = -3;
        }

        // calculate desired velocity based on steering angle
        double velocity;
        if (fabs(angle) >= 0 && fabs(angle) <= 0.15) {
            velocity = 1.5;
        } else if (fabs(angle) <= 0.5) {
            velocity = 1.0;
        } else {
            velocity = 0.5;
        }

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = velocity;

        ackermann_publisher_->publish(drive_msg);
    }

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr drive_msg) {
        
        /*
        Callback function for odom messages. Update current linear velocity based on incoming drive_msg.

        Args:
            drive_msg: Incoming Odom messsage

        Returns:
            None
        */

        // TODO: update current speed
        current_velocity = drive_msg->twist.twist.linear.x;
    
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            scan_msg: Incoming LaserScan message

        Returns:
            None
        */

        // get desired distance parameter
        double dist = (this->get_parameter("dist")).as_double();

        // TODO: replace with error calculated by get_error()
        std::vector<float> laser_readings = scan_msg->ranges;
        ang_min = scan_msg->angle_min;
        ang_max = scan_msg->angle_max;
        ang_increment = scan_msg->angle_increment;
        double curr_error = get_error(laser_readings, dist, current_velocity);

        // update error arrays (past errors)
        integral_error[integral_counter] = curr_error;
        integral_counter = (integral_counter + 1) % 5;

        // TODO: actuate the car with PID
        pid_control(curr_error, prev_error, integral_error);
        
        // update previous error
        prev_error = curr_error;
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
