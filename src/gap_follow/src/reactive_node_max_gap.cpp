#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries

using std::placeholders::_1;

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {

        this->declare_parameter("look_ahead_dist", 20.0);
        this->declare_parameter("bubble_radius", 0.5);

        look_ahead_dist = (this->get_parameter("look_ahead_dist")).as_double();
        bubble_radius = (this->get_parameter("bubble_radius")).as_double();

        /// TODO: create ROS subscribers and publishers
        // subscribers
        laser_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10,
                              std::bind(&ReactiveFollowGap::lidar_callback, this, _1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("ego_racecar/odom", 10,
                              std::bind(&ReactiveFollowGap::drive_callback, this, _1));

        // publishers
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

        // publish this laser scan to visualize in RViz for debugging purpose
        processed_laser_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("processed_scan", 10);
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    
    // processed laser scan publisher for debugging purpose
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr processed_laser_publisher_;

    // define some constants for lidar properties
    const double angle_min = -2.3499999046325684;
    const double angle_max = 2.3499999046325684;
    const double angle_increment = 0.004351851996034384;
    const double pi = 3.14159265358979323846;

    // keep track of current velocity
    double current_velocity;

    // look ahead distance and safety bubble radius, parameters
    double look_ahead_dist;
    double bubble_radius;

    void preprocess_lidar(std::vector<float>& range_data)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        for (int i = 0; i < (int)range_data.size(); i++) {
            double avg_range = range_data[i];
            int beam_num = 1;
            if ((i - 1) > 0) {
                avg_range += range_data[i - 1];
                beam_num++;
            }
            if ((i + 1) < (int)range_data.size()) {
                avg_range += range_data[i + 1];
                beam_num++;
            }
            range_data[i] = avg_range / beam_num;
        }


        // rejecting high values
        for (int i = 0; i < (int)range_data.size(); i++) {
            if (range_data[i] > look_ahead_dist) {
                range_data[i] = look_ahead_dist - 1.0f;
            }
        }
        return;
    }

    void find_max_gap(std::vector<float>& range_data,
                      std::vector<int>& indice)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        bool record_on = false;
        int max_gap_width = 0;
        int curr_gap_start, max_gap_start, max_gap_end;


        for (int i = 0; i < (int)range_data.size(); i++) {
            // if distance is valid AND it is not the last laser scan
            if (range_data[i] > 0.0f && i != (int)(range_data.size() - 1)) {

                // if haven't seen valid gap before, start recording
                if (record_on == false) {
                    // RCLCPP_INFO(this->get_logger(), "start recording at %d", i);
                    record_on = true; // turn on record mode
                    curr_gap_start = i; // record gap start
                }

            } else {
                
                // if seeing invalid point for the first time, end recording
                if (record_on == true) {

                    record_on = false; // turn off recording

                    // check if this gap is the widest one
                    int curr_gap_width = i - curr_gap_start;

                    if (curr_gap_width > max_gap_width) {
                        max_gap_width = curr_gap_width;
                        max_gap_start = curr_gap_start;
                        max_gap_end = i;
                    }
                }

            }
        }

        // push max gap's start and end to vector
        indice.push_back(max_gap_start);
        indice.push_back(max_gap_end);

        return;
    }

    double find_best_point(std::vector<float>& range_data,
                         std::vector<int>& indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        
        // get max gap start and end
        int max_gap_end = indice.back();
        indice.pop_back();
        int max_gap_start = indice.back();

        // keep track of max distance
        double max_dist = 0.0f;
        double best_steering = 0.0f;

        // find the best steering value
        for (int i = max_gap_start; i < max_gap_end; i++) {
            if (range_data[i] > max_dist) {
                max_dist = range_data[i];
                best_steering = i * angle_increment + angle_min;
            }
        }
        
        return best_steering;
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

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // read laser scan
        std::vector<float> laser_readings = scan_msg->ranges;
        
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message 
        preprocess_lidar(laser_readings);

        /// TODO:
        // Find closest point to LiDAR
        double closest_dist = look_ahead_dist;
        int closest_index = 0;

        for (int i = 0; i < (int)laser_readings.size(); i++) {
            if (laser_readings[i] < closest_dist) {
                closest_dist = laser_readings[i];
                closest_index = i;
            }
        }

        // Eliminate all points inside 'bubble' (set them to zero) 
        if (closest_dist < 3.0f) {
            double bubble_radian_range = bubble_radius / closest_dist;
            int bubble_index_range = (int)ceil(bubble_radian_range / angle_increment);
            int bubble_min = ((closest_index - bubble_index_range) < 0) ? 0 : (closest_index - bubble_index_range);
            int bubble_max = ((closest_index + bubble_index_range) >= (int)laser_readings.size()) ?
                            ((int)laser_readings.size() - 1) : (closest_index + bubble_index_range);

            for (int i = bubble_min; i <= bubble_max; i++) {
                laser_readings[i] = 0.0f;
            }
        }

        // publish process lidar scan - for debugging purpose
        auto processed_laser_msg = sensor_msgs::msg::LaserScan();
        processed_laser_msg.header = scan_msg->header;
        processed_laser_msg.angle_min = angle_min;
        processed_laser_msg.angle_max = angle_max;
        processed_laser_msg.angle_increment = angle_increment;
        processed_laser_msg.time_increment = scan_msg->time_increment;
        processed_laser_msg.scan_time = scan_msg->scan_time;
        processed_laser_msg.range_min = scan_msg->range_min;
        processed_laser_msg.range_max = scan_msg->range_max;
        processed_laser_msg.ranges = laser_readings;
        processed_laser_msg.intensities = scan_msg->intensities;
        processed_laser_publisher_->publish(processed_laser_msg);
        
        // Find max length gap
        std::vector<int> indice;
        find_max_gap(laser_readings, indice);

        // Find the best point in the gap
        double best_steering = find_best_point(laser_readings, indice);

        // Publish Drive message
        double throttle;
        
        if (closest_dist < 1.0f) {
            throttle = 0.5f;
        } else if (closest_dist < 3.0f) {
            throttle = 1.0f;
        } else {
            throttle = 2.0f;
        }

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

        drive_msg.drive.steering_angle = best_steering;
        drive_msg.drive.speed = throttle;

        ackermann_publisher_->publish(drive_msg);

    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}