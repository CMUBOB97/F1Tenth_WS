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

// car state space and constraint headers
#include "car_state.h"

/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

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

    // add the publishers and subscribers you need

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;

    // add drive publisher
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    // add path publisher for debug
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // add point publisher for debug
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;

    // add processed laser publisher for debug
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr processed_scan_pub_;

    // add occupancy grid publisher for debug
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

    // add visualization marker array publisher for debug
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_marker_array_pub_;

    // constants for laser properties
    const double PI = 3.1415926536;
    const double angle_min = -2.3499999046325684;
    const double angle_max = 2.3499999046325684;
    const double angle_increment = 0.004351851996034384;

    // constants for tree expansion status
    const int TRAPPED = 0;
    const int ADVANCED = 1;
    const int REACHED = 2;

    // parameters for RRT
    int num_of_samples;
    double max_yaw, max_vel;
    double look_ahead_dist;
    double step_size;
    double goal_sample_rate;
    double angle_sample_range;
    double safety_padding;
    double disparity_extend_range;
    std::string waypoint_file_path;
    std::string waypoint_frame_id;
    std::string pose_to_listen;

    // define transform from/to frames and waypoint index
    std::string fromFrame;
    std::string toFrame;
    int last_waypoint_index;

    // define last heading angle
    double last_steering;

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

    // parameters used for occupancy grid
    int grid_width;
    int grid_height;
    double grid_resolution;

    // vector for processed laser scan
    std::vector<float> processed_scan;

    // callbacks
    // where rrt actually happens
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    // updates last steering command
    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr drive_msg);

    // RRT methods
    int nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point); // TODO move
    int extend(std::vector<RRT_Node> &tree, int nearest_node_index, std::vector<double> &goal_point, bool goal_is_reachable);
    bool check_collision(CarState new_state, CarState prev_state);
    bool is_goal(CarState &state, std::vector<double> &goal_point, bool goal_is_reachable); // TODO move?
    std::vector<RRT_Node> find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node);
    
    // RRT* methods
    double cost(std::vector<RRT_Node> &tree, RRT_Node &node);
    double line_cost(RRT_Node &n1, RRT_Node &n2);
    std::vector<int> near(std::vector<RRT_Node> &tree, RRT_Node &node);

    // helper functions
    void log_waypoints();
    void find_goal_point(std::vector<double> &goal_point);
    void interpolate_points(double x1, double y1, double x2, double y2, std::vector<double> &goal_point);
    void process_scan(std::vector<float>& scan);
    void init_grid();
    void create_marker(RRT_Node &nearest_node, RRT_Node &new_node);
};

