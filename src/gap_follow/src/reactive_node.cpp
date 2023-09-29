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
        /// TODO: create ROS subscribers and publishers

        // declare parameters
        this->declare_parameter("odom_topic_name", "ego_racecar/odom");
        this->declare_parameter("look_ahead_dist", 20.0);
        this->declare_parameter("width_heuristic", 0.5);
        this->declare_parameter("dist_heuristic", 0.5);
        this->declare_parameter("car_width", 0.2032);
        this->declare_parameter("gap_depth_diff", 1.0);
        this->declare_parameter("gap_velocity_ratio", 1.0);
        this->declare_parameter("disparity_threshold", 0.5);
        this->declare_parameter("throttle_level_1", 0.0);
        this->declare_parameter("throttle_level_2", 0.0);
        this->declare_parameter("throttle_level_3", 0.0);

        // read parameters
        std::string odom_topic = (this->get_parameter("odom_topic_name")).as_string();
        look_ahead_dist = (this->get_parameter("look_ahead_dist")).as_double();
        k_width = (this->get_parameter("width_heuristic")).as_double();
        k_dist = (this->get_parameter("dist_heuristic")).as_double();
        car_width = (this->get_parameter("car_width")).as_double();
        gap_depth_diff = (this->get_parameter("gap_depth_diff")).as_double();
        k_gap_v = (this->get_parameter("gap_velocity_ratio")).as_double();
        disp_thres = (this->get_parameter("disparity_threshold")).as_double();
        throttle_lvl_1 = (this->get_parameter("throttle_level_1")).as_double();
        throttle_lvl_2 = (this->get_parameter("throttle_level_2")).as_double();
        throttle_lvl_3 = (this->get_parameter("throttle_level_3")).as_double();

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

    // define some constants for lidar properties
    const double angle_min = -2.3499999046325684;
    const double angle_max = 2.3499999046325684;
    const double angle_increment = 0.004351851996034384;
    const double pi = 3.14159265358979323846;

    // keep track of current velocity
    double current_velocity;

    // different throttle level by parameters
    double throttle_lvl_1, throttle_lvl_2, throttle_lvl_3;

    // look ahead threshold and width/dist heuristics
    // all set with parameters in yaml file
    double look_ahead_dist, car_width, gap_depth_diff, disp_thres;
    double k_width, k_dist, k_gap_v;

    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    
    // processed laser scan publisher for debugging purpose
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr processed_laser_publisher_;

    void preprocess_lidar(std::vector<float>& range_data)
    {   
        /*
         * Preprocess the raw lidar scan value, including:
         * - record disparity and which direction they should extend
         * - extend disparity by half of the car width
         * - throw out values based on current velocity (don't try to hit them)
         */

        // vector recording disparity
        std::vector<size_t> disp_location;

        // record which direction this disparity should extend
        std::vector<bool> disp_to_left;

        // step 0: smoothing over a 3-beam window
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

        // step 1: record disparity
        for (size_t i = 0; i < (range_data.size() - 1); i++) {

            /*
             * rule for disparity evaluation:
             * beam a - beam b (or beam b - beam a)
             * is greater than a threshold
             */

            double laser_diff_1 = range_data[i + 1] - range_data[i];
            double laser_diff_2 = range_data[i] - range_data[i + 1];

            // if beam i + 1 is greater than beam i, extend to left
            if (laser_diff_1 > disp_thres) {
                disp_location.push_back(i);
                disp_to_left.push_back(true);
            } else if (laser_diff_2 > disp_thres) {
                // if beam i is greater than beam i + 1, extend to right
                disp_location.push_back(i + 1);
                disp_to_left.push_back(false);
            }

            // always store the shorter beam's index

        }

        // step 2: while loop, extend disparity
        double half_car_width = car_width / 2;
        double extend_radian_range;
        int extend_index_range;
        size_t disp_index;
        bool disp_is_left;

        while (!disp_location.empty()) {

            /*
             * rule for extending disparity:
             * take the direction to extend
             * calculate extend angle range based on radian = w/2 / lidar
             * extend these laser scan values
             */
            disp_index = disp_location.back();
            disp_location.pop_back();
            disp_is_left = disp_to_left.back();
            disp_to_left.pop_back();

            extend_radian_range = half_car_width / range_data[disp_index];
            extend_index_range = (int)ceil(extend_radian_range / angle_increment);

            if (disp_is_left) {
                int left_end = ((disp_index + extend_index_range) >= range_data.size()) ?
                                (range_data.size() - 1) : (disp_index + extend_index_range);

                for (int j = disp_index + 1; j <= left_end; j++) {
                    range_data[j] = range_data[disp_index];
                }
            } else {
                int right_end = (((int)disp_index - (int)extend_index_range) < 0) ?
                                 0 : (disp_index - extend_index_range); 

                for (int j = disp_index; j >= right_end; j--) {
                    range_data[j] = range_data[disp_index];
                }
            }
        }

        // step 3: threshold distances proportional to velocity and too far away distance
        double gap_dist_threshold = current_velocity * k_gap_v;
        for (size_t k = 0; k < range_data.size(); k++) {
            if (range_data[k] < gap_dist_threshold) {
                range_data[k] = 0.0f;
            } else if (range_data[k] > look_ahead_dist) {
                range_data[k] = look_ahead_dist;
            }
        }

        return;
    }

    void find_gaps(std::vector<float>& range_data,
                   std::vector<int>& gap_width,
                   std::vector<double>& gap_avg_dist,
                   std::vector<double>& gap_com)
    {   
        // Record:
        // - gap width
        // - radian of gap CoM to chase
        // - average distance of the gap
        // of all gaps in free_space_ranges

        // find all gaps
        bool record_on = false;
        int curr_gap_width = 0;
        double curr_gap_avg_dist = 0.0f, curr_gap_com = 0.0f;
        size_t gap_start;

        for (size_t i = 1; i < range_data.size(); i++) {

            // looking for gaps

            // if dist is qualified
            if (range_data[i] > 0.0f) {

                // if a gap spike is observed
                if (fabs(range_data[i] - range_data[i - 1]) > gap_depth_diff) {
                    
                    // if haven't seen valid gap before, start recording
                    if (record_on == false) {
                        record_on = true; // turn on record mode
                    } else {
                        // meaning that currently there is another gap being recorded
                        
                        // save the previous gap first
                        curr_gap_width = i - gap_start;
                        gap_width.push_back(curr_gap_width);

                        for (size_t j = gap_start; j < i; j++) {
                            curr_gap_avg_dist += range_data[j];
                            curr_gap_com += range_data[j] * j;
                        }

                        // calculate com in radians
                        curr_gap_com /= curr_gap_avg_dist; // this is still total distance
                        curr_gap_com = angle_min + (curr_gap_com * angle_increment);
                        
                        curr_gap_avg_dist /= curr_gap_width; //  now it's average distance

                        gap_com.push_back(curr_gap_com);
                        gap_avg_dist.push_back(curr_gap_avg_dist);
                    }

                    // record start, reset calculations
                    gap_start = i;
                    curr_gap_width = 0;
                    curr_gap_avg_dist = 0.0f;
                    curr_gap_com = 0.0f;
                }
            } else {
                // if dist is not qualified

                // if seeing invalid point for the first time, end recording
                if (record_on == true) {

                    record_on = false; // turn off recording

                    // save the data
                    curr_gap_width = i - gap_start;
                    gap_width.push_back(curr_gap_width);

                    for (size_t j = gap_start; j < i; j++) {
                        curr_gap_avg_dist += range_data[j];
                        curr_gap_com += range_data[j] * j;
                    }

                    // calculate com in radians
                    curr_gap_com /= curr_gap_avg_dist; // this is still total distance
                    curr_gap_com = angle_min + (curr_gap_com * angle_increment);
                    
                    curr_gap_avg_dist /= curr_gap_width; //  now it's average distance

                    gap_com.push_back(curr_gap_com);
                    gap_avg_dist.push_back(curr_gap_avg_dist);
                }
            
            }
            
        }


        return;
    }

    double find_best_steering(std::vector<int>& gap_width,
                              std::vector<double>& gap_avg_dist,
                              std::vector<double>& gap_com)
    {   
        // using:
        // - gap width
        // - gap average distance
        // to determine the best steering angle (towards center of mass)

        // score and steering angle of the best gap so far
        double highest_gap_score = 0.0f;
        double best_steering = 0.0f;
        double best_avg_dist = 0.0f;
        int best_width = 0;

        while (!gap_width.empty()) {

            // get those values
            int curr_width = gap_width.back();
            gap_width.pop_back();
            double curr_avg_dist = gap_avg_dist.back();
            gap_avg_dist.pop_back();
            double curr_com = gap_com.back();
            gap_com.pop_back();

            // RCLCPP_INFO(this->get_logger(), "[gap option] width: %d avg_dist: %f com: %f",
            //             curr_width, curr_avg_dist, curr_com);

            // evaluate based on the parameters
            double curr_gap_score = k_width * curr_width + k_dist * curr_avg_dist;

            // compare
            if (curr_gap_score > highest_gap_score) {
                best_steering = curr_com;
                highest_gap_score = curr_gap_score;
                best_avg_dist = curr_avg_dist;
                best_width = curr_width;
            }

        }

        // RCLCPP_INFO(this->get_logger(), "[best] width: %d avg_dist: %f com: %f",
        //                 best_width, best_avg_dist, best_steering);
        
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
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // read laser scan
        std::vector<float> laser_readings = scan_msg->ranges;
        
        // process the range data
        preprocess_lidar(laser_readings);

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

        // vectors and variables for finding best steering
        std::vector<int> gap_width;
        std::vector<double> gap_avg_dist, gap_com;

        // find all gaps
        find_gaps(laser_readings, gap_width, gap_avg_dist, gap_com);

        // find the best one
        double best_steering = find_best_steering(gap_width, gap_avg_dist, gap_com);
        double throttle;
        
        // set throttle based on steering angle
        if (fabs(best_steering) < 0.1) {
            throttle = throttle_lvl_1;
        } else if (fabs(best_steering) < 0.3) {
            throttle = throttle_lvl_2;
        } else {
            throttle = throttle_lvl_3;
        }

        // Publish Drive message
        
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