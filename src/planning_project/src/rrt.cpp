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
    point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("goal_point", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
    processed_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("processed_scan", 10);
    occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", 10);
    vis_marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("vis_marker_array", 10);

    // ROS subscribers
    // TODO: create subscribers as you need
    string pose_topic = "ego_racecar/odom";
    string drive_topic = "drive";
    string scan_topic = "/scan";
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));
    drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic, 1, std::bind(&RRT::drive_callback, this, std::placeholders::_1));
    
    // define parameters for number of sampling
    this->declare_parameter("num_of_samples", 100);
    this->declare_parameter("look_ahead_dist", 3.0);
    this->declare_parameter("step_size", 0.3);
    this->declare_parameter("goal_sample_rate", 0.35);
    this->declare_parameter("angle_sample_range", 1.5708);
    this->declare_parameter("safety_padding", 0.3302);
    this->declare_parameter("disparity_extend_range", 0.1016);
    this->declare_parameter("grid_width", 100);
    this->declare_parameter("grid_height", 200);
    this->declare_parameter("grid_resolution", 0.03);
    this->declare_parameter("max_yaw", PI/4);
    this->declare_parameter("max_vel", 6.0);
    this->declare_parameter("waypoint_file_path", "/sim_ws/src/planning_project/waypoints/waypoints.txt");
    this->declare_parameter("waypoint_frame_id", "map");
    this->declare_parameter("pose_to_listen", "ego_racecar/base_link");

    num_of_samples = (this->get_parameter("num_of_samples")).as_int();
    look_ahead_dist = (this->get_parameter("look_ahead_dist")).as_double();
    step_size = (this->get_parameter("step_size")).as_double();
    goal_sample_rate = (this->get_parameter("goal_sample_rate")).as_double();
    angle_sample_range = (this->get_parameter("angle_sample_range")).as_double();
    safety_padding = (this->get_parameter("safety_padding")).as_double();
    disparity_extend_range = (this->get_parameter("disparity_extend_range")).as_double();
    grid_width = (this->get_parameter("grid_width")).as_int();
    grid_height = (this->get_parameter("grid_height")).as_int();
    grid_resolution = (this->get_parameter("grid_resolution")).as_double();
    max_yaw = (this->get_parameter("max_yaw")).as_double();
    max_vel = (this->get_parameter("max_vel")).as_double();
    waypoint_file_path = (this->get_parameter("waypoint_file_path")).as_string();
    waypoint_frame_id = (this->get_parameter("waypoint_frame_id")).as_string();
    pose_to_listen = (this->get_parameter("pose_to_listen")).as_string();

    MAX_YAW = max_yaw;
    MAX_VEL = max_vel;
    
    GRID_RESOLUTION = grid_resolution;
    GRID_WIDTH = grid_width;
    GRID_HEIGHT = grid_height;
    
    GOAL_SAMPLE_RATE = goal_sample_rate;

    // create path using waypoints in the file
    log_waypoints();

    // initialize the occupancy grid
    init_grid();

    // transform listener with buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // initialize last waypoint index
    last_waypoint_index = -1;

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

void RRT::drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr drive_msg) {
    last_steering = drive_msg->drive.steering_angle;
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    // acquire the laser scan
    std::vector<float> laser_values = scan_msg->ranges;

    // set occupancy grid time stamp and reference frame
    rrt_grid.header.stamp = this->get_clock()->now();
    rrt_grid.header.frame_id = pose_to_listen;

    // reset occupancy grid
    std::fill(rrt_grid.data.begin(), rrt_grid.data.end(), 0);

    // update occupancy grid
    for (int i = 0; i < grid_height; i++) {
        for (int j = 0; j < grid_width; j++) {

            // find x, y relative to the car
            double x_cell = grid_resolution * j;
            double y_cell = grid_resolution * (i - grid_height / 2);

            // find euclidean distance
            double dist_cell = euclidean_dist(x_cell, y_cell);

            // find angle
            double angle_cell = atan2(y_cell, x_cell);
            int angle_index = (angle_cell - angle_min) / angle_increment;

            // check if distance match
            if (dist_cell >= laser_values[angle_index] - safety_padding) {
                rrt_grid.data[i * grid_width + j] = 100;
            }
        }
    }

    // publish that
    occupancy_grid_pub_->publish(rrt_grid);

    // process it
    process_scan(laser_values);

    // publish the processed scan for debugging purpose
    auto processed_laser_msg = sensor_msgs::msg::LaserScan();
    processed_laser_msg.header = scan_msg->header;
    processed_laser_msg.angle_min = angle_min;
    processed_laser_msg.angle_max = angle_max;
    processed_laser_msg.angle_increment = angle_increment;
    processed_laser_msg.time_increment = scan_msg->time_increment;
    processed_laser_msg.scan_time = scan_msg->scan_time;
    processed_laser_msg.range_min = scan_msg->range_min;
    processed_laser_msg.range_max = scan_msg->range_max;
    processed_laser_msg.ranges = laser_values;
    processed_laser_msg.intensities = scan_msg->intensities;
    processed_scan_pub_->publish(processed_laser_msg);
}

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    // tree as std::vector
    std::vector<RRT_Node> tree = {};

    // insert the first tree node
    RRT_Node root;
    root.state.x = 0.0f;
    root.state.y = 0.0f;
    root.state.t = 0.0f;
    root.state.yaw = 0.0f;
    root.state.vel = pose_msg->twist.twist.linear.x;
    root.state.alpha = last_steering;
    root.state.accel = 0.0f;
    root.state.delta_alpha = 0.0f;
    root.cost = 0.0f;
    root.parent = -1;
    root.is_root = true;
    tree.push_back(root);

    // clear rrt marker array and reset marker id in a new iteration
    rrt_marks.markers.clear();
    marker_id = 0;

    // step 0: publish path and update grid origin pose
    path_msg.header.stamp = this->get_clock()->now();
    path_pub_->publish(path_msg);

    // step 1: find the goal direction with fixed look ahead distance
    std::vector<double> goal = {-look_ahead_dist, 0.0f};
    bool goal_status = false;
    find_goal_point(goal);
    
    if (goal[0] > 0.0f) {
        goal_status = true;
    }

    // keep the minimum distance to goal to determine the best action to pick
    double best_dist = euclidean_dist(goal[0], goal[1]);
    RRT_Node best_node = root;

    std::vector<double> point = {0.0f, 0.0f};

    // step 2: given updated occupancy grid, sample angles and expand the tree (record furtherest route)
    // during sampling, expand occupancy grid (ros vis marker array)
    for (int i = 0; i < num_of_samples; i++) {

        // sample a point
        std::vector<double> random_point = sample_config(goal, goal_status);

        // find the nearest neighbor in the tree
        int nearest_index = nearest(tree, random_point);

        // expand that node
        int expand_status = extend(tree, nearest_index, goal, goal_status);

        // check expand status
        if (expand_status == REACHED) {
            break;
        }

        // check the distance to best dist
        if (expand_status != TRAPPED) {
            point[0] = tree.back().state.x;
            point[1] = tree.back().state.y;
            double curr_dist = euclidean_dist(goal, point);
            if (curr_dist < best_dist) {
                best_node = tree.back();
                best_dist = curr_dist;
            }
        }
    }

    // for debug purpose, publish the visualization marker array
    vis_marker_array_pub_->publish(rrt_marks);


    // step 3: recover the path
    // - first, select the path to goal
    // - second, (if goal is blocked) select the path that get to the look ahead distance
    // - third, (if iterations are exhausted) select the recorded furtherest path
    std::vector<RRT_Node> path_points = find_path(tree, best_node);

    // step 4: command steering
    // strategy: use the first steering
    if (path_points.size() >= 2) {
        RRT_Node steering_node = path_points[path_points.size() - 2];
        double steering_angle = steering_node.state.alpha;
        double velocity = steering_node.state.vel;

        double time_scaling = 1.0 / (DT * EXTEND_STEP) / 100.0;
        double steering_diff = steering_angle - last_steering;
        double change_in_steering = steering_diff * time_scaling;
        double vel_diff = velocity - pose_msg->twist.twist.linear.x;
        double change_in_vel = vel_diff * time_scaling * 2.0;

        // command drive
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = last_steering + change_in_steering;
        drive_msg.drive.steering_angle_velocity = MAX_STEERING_CHANGE;
        drive_msg.drive.speed = pose_msg->twist.twist.linear.x + change_in_vel;
        drive_msg.drive.jerk = 0.0;
        drive_msg.drive.acceleration = MAX_GAS_ACCEL;

        // RCLCPP_INFO(this->get_logger(), ", %f, %f, %f, %f, %f, %f",
        //                 pose_msg->pose.pose.position.x,
        //                 pose_msg->pose.pose.position.y,
        //                 drive_msg.drive.speed,
        //                 drive_msg.drive.acceleration,
        //                 drive_msg.drive.steering_angle,
        //                 drive_msg.drive.steering_angle_velocity);

        drive_pub_->publish(drive_msg);
    }
}

int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<RRT_Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    int nearest_node = 0;
    double best_dist = -1.0f;
    
    // loop through the tree to find the nearest node
    for (size_t i = 0; i < tree.size(); i++) {
        double dist = car_state_distance(tree[i], sampled_point);
        // update
        if (best_dist < 0 || dist < best_dist) {
            best_dist = dist;
            nearest_node = i;
        }
    }

    return nearest_node;
}

int RRT::extend(std::vector<RRT_Node> &tree, int nearest_node_index, std::vector<double> &goal_point, bool goal_status) {

    // The function extend(tree, nearest, sample) attempts to add a new node
    // between nearest node and sampled point based on step size
    // by collision check, it will return a status
    // if TRAPPED, new node will not be added to the tree (hit an obstacle)
    // if ADVANCED, new node is added to the tree, but goal is not reached yet
    // if REACHED, new node is added to the tree, and goal is reached

    // increment based on the step size
    RRT_Node new_node;

    // check collision variable
    bool collision_status = false;
    bool goal_reached = false;
    bool kin_feasible = true;

    std::pair<double, double> action = sample_action();
    double d_alpha = action.first;
    double accel = action.second;

    // Construct current car state
    CarState state_sim = tree[nearest_node_index].state;
    CarState state_sim_next;

    // forward simulate towards the goal state
    for (int i = 0; i < EXTEND_STEP; i++) {
        state_sim_next = CalcNextState(state_sim, accel, d_alpha);

        collision_status = check_collision(state_sim, state_sim_next);
        kin_feasible = is_kinematically_feasible(state_sim_next);
        if (collision_status || !kin_feasible) {
            return TRAPPED;
        }

        goal_reached = is_goal(new_node, goal_point, false);
        new_node.state = state_sim_next;
        if (goal_reached) {
            break;
        }

        // update state
        state_sim = state_sim_next;

    }

    // if no collision occurs, add the node
    new_node.parent = nearest_node_index;
    new_node.is_root = false;
    tree.push_back(new_node);

    // add visualization mark here
    create_marker(tree[nearest_node_index], new_node);

    if (goal_reached) {
        return REACHED;
    } else {
        return ADVANCED;
    }

}

bool RRT::check_collision(CarState new_state, CarState prev_state) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    // find nearest and new nodes' indices in occupancy grid
    double prev_grid_x = prev_state.x / grid_resolution;
    double prev_grid_y = prev_state.y / grid_resolution + grid_height / 2;
    double new_grid_x = new_state.x / grid_resolution;
    double new_grid_y = new_state.y / grid_resolution + grid_height / 2;

    int start_x = floor((prev_grid_x < new_grid_x) ? prev_grid_x : new_grid_x);
    int end_x = ceil((prev_grid_x < new_grid_x) ? new_grid_x : prev_grid_x);
    int start_y = floor((prev_grid_y < new_grid_y) ? prev_grid_y : new_grid_y);
    int end_y = ceil((prev_grid_y < new_grid_y) ? new_grid_y : prev_grid_y);

    // draw a square on occupancy grid between these two points
    // check for occupancy within this box
    for (int i = start_y; i < end_y; i++) {
        for (int j = start_x; j < end_x; j++) {
            int index = i * grid_width + j;
            if (rrt_grid.data[index] > 50) {
                return true;
            }
        }
    }
    return false;
}

bool RRT::is_goal(RRT_Node &latest_added_node, std::vector<double> &goal, bool goal_status) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal (double vector): vector of goal point {x, y}
    //   goal_status (bool): if x and y are not goals, just check for how far latest_added_node has expanded
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method
    
    // the case where goal is valid
    if (goal_status) {
        double x_diff = fabs(latest_added_node.state.x - goal[0]);
        double y_diff = fabs(latest_added_node.state.y - goal[1]);
        // TODO: if we need to check yaw and velocity difference, come back later
        if (x_diff + y_diff <= 1e-3) {
            close_enough = true;
        }
    } else { // when goal is invalid, check for distance expanded
        double expand_dist = euclidean_dist(latest_added_node.state.x, latest_added_node.state.y);
        // TODO: if we need to check yaw and velocity difference, come back later
        if (expand_dist > look_ahead_dist) {
            close_enough = true;
        }
    }

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

    CarState state_next, state_current;
    
    RRT_Node search_node = latest_added_node;

    // push the goal of the path first
    found_path.push_back(search_node);

    while (search_node.is_root == false) {
        search_node = tree[search_node.parent];
        found_path.push_back(search_node);
    }

    RCLCPP_INFO(this->get_logger(), "------------------------- showing a new path -------------------------");
    RCLCPP_INFO(this->get_logger(), ", time, x, y, yaw, velocity, steering, throttle, steering velocity");
    
    // for (int i = found_path.size() - 1; i >= 1; i--) {
    //     state_current = found_path[i].state;
    //     state_next = found_path[i - 1].state;
    //     RCLCPP_INFO(this->get_logger(), ", %f, %f, %f, %f, %f, %f, %f, %f",
    //                 state_current.t, state_current.x, state_current.y, state_current.yaw,
    //                 state_current.vel, state_current.alpha, state_next.accel, state_next.delta_alpha);
    //     for (int j = 0; j < EXTEND_STEP - 1; j++) {
    //         state_next = CalcNextState(state_current, state_next.accel, state_next.delta_alpha);
    //         RCLCPP_INFO(this->get_logger(), ", %f, %f, %f, %f, %f, %f, %f, %f",
    //                 state_next.t, state_next.x, state_next.y, state_next.yaw,
    //                 state_next.vel, state_next.alpha, state_next.accel, state_next.delta_alpha);
    //         state_current = state_next;
    //     }
    // }

    return found_path;
}

// ---------------------- begin RRT* methods ----------------------

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

// ---------------------- end RRT* methods ----------------------

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

    // define path_msg relative frame
    path_msg.header.frame_id = waypoint_frame_id;

    // update path length
    path_length = (int)path_msg.poses.size();
}

void RRT::find_goal_point(std::vector<double> &goal_point) {
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
        return;
    }

    // find the current waypoint using interpolation method
    int start_search_index = (last_waypoint_index == -1) ? 0 : last_waypoint_index;
    int search_count = 0;

    // set heading found status
    bool heading_found = false;
    double heading;

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
            heading_found = true;
            break;
        } else {
            search_count++;
        }
    }

    if (heading_found) {
        // run interpolation to update goal location
        interpolate_points(first_transformed.pose.position.x,
                           first_transformed.pose.position.y,
                           second_transformed.pose.position.x,
                           second_transformed.pose.position.y,
                           goal_point);
    }

    return;
}

void RRT::interpolate_points(double x1, double y1, double x2, double y2, std::vector<double> &goal_point) {

    // interpolate between these two waypoints
    // to find the waypoint car needs to follow
    // return the heading angle of that interpolated
    // goal point

    // assuming that x1, y1 are within the radius, and x2, y2 are outside the radius
    double x_diff = x2 - x1;
    double y_diff = y2 - y1;
    double a = x_diff * x_diff + y_diff * y_diff;
    double b = 2 * (x_diff * x1 + y_diff * y1);
    double c = x1 * x1 + y1 * y1 - look_ahead_dist * look_ahead_dist;

    // get interpolation fraction
    // set to 0 avoid algebra error
    double quad_term = sqrt(max(b * b - 4 * a * c, 0.0));
    double quad_sln = (-b + quad_term) / 2;
    
    // solution check, if cannot find a valid solution, set to second goal point
    double frac = (quad_sln <= 1 && quad_sln >= 0) ? quad_sln : 1.0;

    // get goal x and y
    goal_point[0] = x1 + frac * x_diff;
    goal_point[1] = y1 + frac * y_diff;

    // publish that as a point, check correctness on rviz
    auto point_msg = geometry_msgs::msg::PointStamped();
    point_msg.header.stamp = this->get_clock()->now();
    point_msg.header.frame_id = pose_to_listen;
    point_msg.point.x = goal_point[0];
    point_msg.point.y = goal_point[1];
    point_msg.point.z = 0;
    point_pub_->publish(point_msg);

    return;
}

void RRT::process_scan(std::vector<float>& scan) {

    // process the laser scan to
    // - buffer the walls by car's length
    // - extend disparities by car's width
    // this creates a occupancy grid (configuration space)
    // for car to traverse through

    for (size_t i = 0; i < scan.size(); i++) {

        // buffer by car's length
        scan[i] -= safety_padding;

    }

    // (optional) TODO: smooth laser readings over a window (3 or 5)

    // extend disparities observed
}

void RRT::init_grid() {

    // pose to be updated and used for occupancy grid origin
    geometry_msgs::msg::Pose grid_origin;

    // initialize other infos
    // note that grid width is along x (in front of car base link)
    // and grid height is along y (parallel to car base link)
    rrt_grid.info.map_load_time = this->get_clock()->now();
    rrt_grid.info.resolution = grid_resolution;
    rrt_grid.info.width = grid_width;
    rrt_grid.info.height = grid_height;
    
    // initialize the occupancy grid for rrt
    grid_origin.position.x = 0.0f;
    grid_origin.position.y = -grid_resolution * grid_height / 2;
    grid_origin.position.z = 0.0f;
    grid_origin.orientation.x = 0.0f;
    grid_origin.orientation.y = 0.0f;
    grid_origin.orientation.z = 0.0f;
    grid_origin.orientation.w = 1.0f;
    rrt_grid.info.origin = grid_origin;

    // initialize the data field with correct number of elements
    int num_of_cells = grid_width * grid_height;
    for (int i = 0; i < num_of_cells; i++) {
        rrt_grid.data.push_back((int8_t)0);
    }

}

void RRT::create_marker(RRT_Node &nearest_node, RRT_Node &new_node) {

    // this helper function will add new nodes of the tree
    // to visualization marker array as a line list

    // initialize marker
     visualization_msgs::msg::Marker new_mark;

    // initialize frames
    new_mark.header.frame_id = pose_to_listen;
    new_mark.header.stamp = this->get_clock()->now();
    new_mark.ns = "my_rrt";
    new_mark.id = marker_id;
    new_mark.type = visualization_msgs::msg::Marker::LINE_LIST;
    new_mark.action = visualization_msgs::msg::Marker::ADD;
    new_mark.scale.x = 0.1;
    new_mark.scale.y = 0.1;
    new_mark.scale.z = 0.1;
    new_mark.color.a = 1.0;
    new_mark.color.r = 1.0 - (nearest_node.state.vel / MAX_VEL);
    new_mark.color.g = 0.0;
    new_mark.color.b = nearest_node.state.vel / MAX_VEL;
    new_mark.lifetime = rclcpp::Duration::from_seconds(0.1);

    // push the points in the array
    geometry_msgs::msg::Point start_point, end_point;
    start_point.x = nearest_node.state.x;
    start_point.y = nearest_node.state.y;
    start_point.z = 0.0f;
    end_point.x = new_node.state.x;
    end_point.y = new_node.state.y;
    end_point.z = 0.0f;
    new_mark.points.push_back(start_point);
    new_mark.points.push_back(end_point);

    // push the marker in
    rrt_marks.markers.push_back(new_mark);

    // increment marker_id
    marker_id++;

    return;
}