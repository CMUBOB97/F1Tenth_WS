// RRT assignment

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

// Struct defining the car state model used in RRT tree.
// use time, x, y, heading (yaw), velocity, and steering (alpha)
typedef struct CarState {
    double t;
    double x;
    double y;
    double yaw;
    double vel;
    double alpha; 
} CarState;

// Struct defining the RRT_Node object in the RRT tree.
// defined in polar coordinate fashion
// distance to current position and angle
typedef struct RRT_Node {
    CarState state; // current car state
    double cost; // cost to parent, can be used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} RRT_Node;

class RRT : public rclcpp::Node {
public:
    RRT();
    virtual ~RRT();
private:

    // ----------------------------- BEGIN PUBLISHER/SUBSCRIPTION -----------------------------

    // add the following subscribers:
    // - pose subscriber for pose callback (rrt routine)
    // - scan subscriber for scan callback (occupancy grid update)
    // - drive subscriber for drive callback (steering angle update for kinematics)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;

    // add the following publishers:
    // - drive publisher
    // - path publisher for waypoint debug
    // - point publisher for goal point debug
    // - processed laser publisher for preprocessing debug (OPTIONAL)
    // - occupancy grid publisher for rrt grid debug
    // - visualization marker array publisher for rrt tree node debug
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr processed_scan_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_marker_array_pub_;

    // ----------------------------- END PUBLISHER/SUBSCRIPTION -----------------------------

    // ----------------------------- BEGIN RANDOM GENERATOR -----------------------------
    
    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> sample_type;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

    // ----------------------------- END RANDOM GENERATOR -----------------------------

    // ----------------------------- BEGIN CONSTANTS -----------------------------

    // constants for laser properties
    const double PI = 3.1415926536;
    const double wheel_base = 0.3302;
    const double angle_min = -2.3499999046325684;
    const double angle_max = 2.3499999046325684;
    const double angle_increment = 0.004351851996034384;

    // constants for tree expansion status
    const int TRAPPED = 0;
    const int ADVANCED = 1;
    const int REACHED = 2;

    // ----------------------------- END CONSTANTS -----------------------------

    // ----------------------------- BEGIN PARAMETERS -----------------------------

    // parameters for RRT
    int num_of_samples;
    double look_ahead_dist;
    double goal_sample_rate;

    // parameters for kinematic constraints
    double max_yaw, max_vel;
    double max_steering;
    double steering_gain, velocity_gain;
    double time_step;
    int extend_step;

    // parameters for occupancy grid
    int grid_width;
    int grid_height;
    double grid_resolution;
    int valid_path_length;
    double safety_padding;
    double disparity_extend_range;

    // parameters for waypoints
    std::string waypoint_file_path;
    std::string waypoint_frame_id;
    std::string pose_to_listen;

    // parameters for topic name
    std::string pose_topic;

    // parameters for drive dynamics
    double max_accel, max_jerk;
    double temporal_filter_ratio;

    // ----------------------------- END PARAMETERS -----------------------------

    // ----------------------------- BEGIN INTERNAL VARIABLES -----------------------------

    // define last heading angle to be paired with drive sub
    double last_velocity;
    double last_steering;

    // define transform from/to frames and waypoint index
    std::string fromFrame;
    std::string toFrame;
    int last_waypoint_index;

    // listener to listen updates on pose_to_listen
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // data structure to store
    // - path file
    // - path
    // - occupancy grid
    // - rrt visualization
    ifstream waypoint_file;
    nav_msgs::msg::Path path_msg;
    int path_length;
    nav_msgs::msg::OccupancyGrid rrt_grid;
    visualization_msgs::msg::MarkerArray rrt_marks;
    int marker_id;

    // TODO: vector for processed laser scan, for better obstacle avoidance
    std::vector<float> processed_scan;

    // ----------------------------- END INTERNAL VARIABLES -----------------------------

    // ----------------------------- BEGIN FUNCTIONS -----------------------------

    // callbacks
    // where rrt actually happens
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    // updates last steering command
    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr drive_msg);

    // RRT methods
    std::vector<double> sample_config(std::vector<double> &goal, bool goal_status);
    int nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point);
    int extend(std::vector<RRT_Node> &tree, int nearest_node_index, std::vector<double> &sampled_point, std::vector<double> &goal_point, bool goal_status);
    bool check_collision(CarState new_state, CarState prev_state);
    bool is_goal(RRT_Node &latest_added_node, std::vector<double> &goal_point, bool goal_status);
    std::vector<RRT_Node> find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node);

    // helper functions
    void log_waypoints();
    void find_goal_point(std::vector<double> &goal_point);
    void interpolate_points(double x1, double y1, double x2, double y2, std::vector<double> &goal_point);
    double euclidean_dist(double x, double y);
    double euclidean_dist(std::vector<double> v1, std::vector<double> v2);
    void process_scan(std::vector<float>& scan);
    void init_grid();
    void create_marker(RRT_Node &nearest_node, RRT_Node &new_node);
    CarState CalcNextState(CarState state, double accel, double delta_alpha);
    
    // ----------------------------- END FUNCTIONS -----------------------------
};

