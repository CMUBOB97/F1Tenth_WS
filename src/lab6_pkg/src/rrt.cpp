// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()) {

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

    // ROS subscribers
    // TODO: create subscribers as you need
    string pose_topic = "ego_racecar/odom";
    string scan_topic = "/scan";
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));
    
    // define parameters for number of sampling
    this->declare_parameter("num_of_samples", 100);
    this->declare_parameter("look_ahead_dist", 3.0);
    this->declare_parameter("step_size", 0.3);
    this->declare_parameter("angle_sample_range", 1.5708);
    this->declare_parameter("safety_padding", 0.3302);
    this->declare_parameter("disparity_extend_range", 0.1016);
    this->declare_parameter("waypoint_file_path", "/sim_ws/waypoints.txt");
    this->declare_parameter("waypoint_frame_id", "map");
    this->declare_parameter("pose_to_listen", "ego_racecar/base_link");

    num_of_samples = (this->get_parameter("num_of_samples")).as_int();
    look_ahead_dist = (this->get_parameter("look_ahead_dist")).as_double();
    step_size = (this->get_parameter("step_size")).as_double();
    angle_sample_range = (this->get_parameter("angle_sample_range")).as_double();
    safety_padding = (this->get_parameter("safety_padding")).as_double();
    disparity_extend_range = (this->get_parameter("disparity_extend_range")).as_double();
    waypoint_file_path = (this->get_parameter("waypoint_file_path")).as_string();
    waypoint_frame_id = (this->get_parameter("waypoint_frame_id")).as_string();
    pose_to_listen = (this->get_parameter("pose_to_listen")).as_string();

    // create path using waypoints in the file
    log_waypoints();

    // initialize the occupancy grid (resize the vector)
    rrt_marks.markers.resize(num_of_samples);
    
    // initialize RRT data structure (resize the vector)
    rrt.resize(num_of_samples);

    // transform listener with buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // initialize last waypoint index
    last_waypoint_index = -1;

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    // TODO: update your occupancy grid
}

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    // tree as std::vector
    std::vector<RRT_Node> tree;

    // TODO: fill in the RRT main loop



    // path found as Path message

}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)
    
    return sampled_point;
}


int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<RRT_Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    // TODO: fill in this method

    return nearest_node;
}

RRT_Node RRT::steer(RRT_Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (RRT_Node): new node created from steering

    RRT_Node new_node;
    // TODO: fill in this method

    return new_node;
}

bool RRT::check_collision(RRT_Node &nearest_node, RRT_Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    // TODO: fill in this method

    return collision;
}

bool RRT::is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method

    return close_enough;
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<RRT_Node> found_path;
    // TODO: fill in this method

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<RRT_Node>): the current tree
    //    node (RRT_Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

double RRT::line_cost(RRT_Node &n1, RRT_Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (RRT_Node): the RRT_Node at one end of the path
    //    n2 (RRT_Node): the RRT_Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

std::vector<int> RRT::near(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current tree
    //   node (RRT_Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}

void RRT::log_waypoints() {
    // use the waypoint file to fill waypoints in the path message
    // for finding the next waypoint
    
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

    // update path length
    path_length = (int)path_msg.poses.size();
}

double RRT::find_goal_point() {
    // given Path msg, loop through it
    // to find the desired goal direction
    // return type is double, the angle of goal
    // point relative to current car pose

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
        return -PI;
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
        bool shorter = (euclidean_dist(first_transformed.pose.position.x, first_transformed.pose.position.y) <= look_ahead_dist);
        bool longer = (euclidean_dist(second_transformed.pose.position.x, second_transformed.pose.position.y) > look_ahead_dist);

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
    double heading = interpolate_points(first_transformed.pose.position.x,
                                        first_transformed.pose.position.y,
                                        second_transformed.pose.position.x,
                                        second_transformed.pose.position.y);

    return heading;
}

double RRT::interpolate_points(double x1, double y1, double x2, double y2) {

    // interpolate between these two waypoints
    // to find the waypoint car needs to follow
    // return the heading angle of that interpolated
    // goal point

    // to be done
    return 0.0f;
}

double RRT::euclidean_dist(double x, double y) {

    // calculate the euclidean distance to (x, y)
    // assuming from (0, 0)
    // return dist

    return sqrt(x* x + y * y);
}