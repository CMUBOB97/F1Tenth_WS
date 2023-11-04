#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!

private:

    // file path string
    std::string waypoint_file_path;

    // input file handler
    ifstream waypoint_file;

    // waypoint frame id
    std::string waypoint_frame_id;

    // path message created by waypoints
    nav_msgs::msg::Path path_msg;
    int path_length;

    // topic name for tf listener to listen
    std::string pose_to_listen;

    // path publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    
    // drive publisher
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;

    // create listener to listen updates on ego_racecar/base_link
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // time-based listener
    rclcpp::TimerBase::SharedPtr timer_;
    int time_interval;

    // index for last waypoint index
    int last_waypoint_index;

    // pure pursuit look ahead distance and forward velocity
    double look_ahead_dist;
    double forward_velocity;
    double steering_scale;
    double max_steering;

    // define transform from and to frames
    std::string fromFrame;
    std::string toFrame;

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        // TODO: create ROS subscribers and publishers

        // declare parameters
        this->declare_parameter("waypoint_file_path", "/sim_ws/waypoints.txt");
        this->declare_parameter("waypoint_frame_id", "map");
        this->declare_parameter("pose_to_listen", "ego_racecar/base_link");
        this->declare_parameter("look_ahead_dist", 1.5);
        this->declare_parameter("forward_velocity", 1.0);
        this->declare_parameter("max_steering", 1.0);
        this->declare_parameter("steering_scale", 0.25);
        this->declare_parameter("callback_interval", 1);

        // read parameters
        waypoint_file_path = (this->get_parameter("waypoint_file_path")).as_string();
        waypoint_frame_id = (this->get_parameter("waypoint_frame_id")).as_string();
        pose_to_listen = (this->get_parameter("pose_to_listen")).as_string();
        look_ahead_dist = (this->get_parameter("look_ahead_dist")).as_double();
        forward_velocity = (this->get_parameter("forward_velocity")).as_double();
        max_steering = (this->get_parameter("max_steering")).as_double();
        steering_scale = (this->get_parameter("steering_scale")).as_double();
        time_interval = (this->get_parameter("callback_interval")).as_int();

        // open the file
        waypoint_file.open(waypoint_file_path);

        // extract waypoints
        std::string line;
        while (std::getline(waypoint_file, line)) {
            
            // read x, y, z values
            float x, y, z;
            std::stringstream ss(line);
            ss >> x;
            ss >> y;
            ss >> z;

            // write x, y, z into stamped pose
            geometry_msgs::msg::PoseStamped new_pose;
            new_pose.header.stamp = this->get_clock()->now();
            new_pose.header.frame_id = waypoint_frame_id;
            new_pose.pose.position.x = x;
            new_pose.pose.position.y = y;
            new_pose.pose.position.z = z;

            // insert new pose into the vector
            path_msg.poses.push_back(new_pose);

        }

        // close the file
        RCLCPP_INFO(this->get_logger(), "waypoints loaded");
        waypoint_file.close();

        // create path publisher
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", 10);

        // create ackermann publisher
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

        // transform listener with buffer
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // create timer
        timer_ = this->create_wall_timer(100ms, std::bind(&PurePursuit::pursuit_callback, this));

        // initialize last waypoint index and path length
        last_waypoint_index = -1;
        path_length = (int)path_msg.poses.size();
    }

    // helper function to calculate euclidean distance between two points
    double euclidean_dist(double x1, double y1, double x2, double y2)
    {
        double x_diff = fabs(x1 - x2);
        double y_diff = fabs(y1 - y2);
        return sqrt(x_diff * x_diff + y_diff * y_diff);
    }

    // helper function to run interpolation between two x, y pairs
    double interpolate_points(double x1, double y1, double x2, double y2)
    {
        // this can be solved with quadratic formula
        // assuming that x1, y1 are within the radius, and x2, y2 are outside the radius
        double x_diff = x2 - x1;
        double y_diff = y2 - y1;
        double a = x_diff * x_diff + y_diff * y_diff;
        double b = 2 * (x_diff + y_diff);
        double c = x1 * x1 + y1 * y1 - look_ahead_dist * look_ahead_dist;

        // get interpolation fraction
        // set to 0 avoid algebra error
        double frac = (-b + sqrt(max(b * b - 4 * a * c, 0.0))) / 2;

        // return y component
        return y1 + frac * y_diff;

    }

    void pursuit_callback()
    {
        // publish path msg
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = waypoint_frame_id;
        path_publisher_->publish(path_msg);

        // try to obtain current map to base_link transform
        fromFrame = waypoint_frame_id;
        toFrame = pose_to_listen;
        geometry_msgs::msg::TransformStamped curr_t;
        try{
            curr_t = tf_buffer_->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                toFrame.c_str(), fromFrame.c_str(), ex.what());
            return;
        }

        // find the current waypoint using interpolation method
        int start_search_index = (last_waypoint_index == -1) ? 0 : last_waypoint_index;
        int search_count = 0;

        // name input and output poses
        geometry_msgs::msg::PoseStamped first_pose, second_pose;
        geometry_msgs::msg::PoseStamped first_transformed, second_transformed;
        
        // use search_count to prevent loop around not recognized
        for (int i = start_search_index; search_count < path_length; i++) {
            // be careful of wrap around
            i = (i >= path_length) ? 0 : i;

            // two frame indices
            int first_frame_index = i;
            int second_frame_index = ((first_frame_index + 1) >= path_length) ? 0 : (first_frame_index + 1);

            // take out two poses
            first_pose = path_msg.poses[first_frame_index];
            second_pose = path_msg.poses[second_frame_index];

            // transform waypoints to baselink frame
            tf2::doTransform(first_pose, first_transformed, curr_t);
            tf2::doTransform(second_pose, second_transformed, curr_t);

            // find two waypoints, one shorter, one longer for interpolation use
            bool shorter = (euclidean_dist(first_transformed.pose.position.x, first_transformed.pose.position.y, 0.0, 0.0) <= look_ahead_dist);
            bool longer = (euclidean_dist(second_transformed.pose.position.x, second_transformed.pose.position.y, 0.0, 0.0) > look_ahead_dist);

            // found the transform pairs for interpolation
            // - first frame is within the radius
            // - second frame is beyond the radius
            // - both first and second frames are in front of the car
            if (shorter && longer && first_transformed.pose.position.x > 0.0f && second_transformed.pose.position.x > 0.0f) {
                last_waypoint_index = first_frame_index;
                break;
            } else {
                search_count++;
            }
        }

        // run interpolation
        double y_heading = interpolate_points(first_transformed.pose.position.x,
                                              first_transformed.pose.position.y,
                                              second_transformed.pose.position.x,
                                              second_transformed.pose.position.y);

        // calculate curvature/steering angle and velocity (pure pursuit)
        double steering = steering_scale * 2 * y_heading / (look_ahead_dist * look_ahead_dist);

        // limit steering
        steering = (steering > max_steering) ? max_steering : steering;
        steering = (steering < -max_steering) ? -max_steering : steering;

        // velocity scaled by steering
        double velocity = forward_velocity / (0.75 + fabs(steering) * 1.25);

        // TODO: publish drive message, don't forget to limit the steering angle.
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = steering;
        drive_msg.drive.speed = velocity;

        ackermann_publisher_->publish(drive_msg);
    }

    ~PurePursuit() {}
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}