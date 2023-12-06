// This file contains the class definition of tree nodes and RRT_ROS_Node
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt.h"
#include "planner.h"

// Destructor of the RRT_ROS_Node class
RRT_ROS_Node::~RRT_ROS_Node()
{
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT_ROS_Node"), "%s\n", "RRT_ROS_Node shutting down");
}

// Constructor of the RRT_ROS_Node class
RRT_ROS_Node::RRT_ROS_Node() : rclcpp::Node("rrt_node"), gen((std::random_device())())
{

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
    drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic, 1, std::bind(&RRT_ROS_Node::drive_callback, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        pose_topic, 1, std::bind(&RRT_ROS_Node::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 1, std::bind(&RRT_ROS_Node::scan_callback, this, std::placeholders::_1));

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
    this->declare_parameter("max_yaw", PI / 4);
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

    // create path using waypoints in the file
    log_waypoints();

    // initialize the occupancy grid
    init_grid();

    // transform listener with buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // initialize last waypoint index
    last_waypoint_index = -1;

    RCLCPP_INFO(rclcpp::get_logger("RRT_ROS_Node"), "%s\n", "Created new RRT_ROS_Node Object.");
}
// The drive callback, update last steering angle commanded here
void RRT_ROS_Node::drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr drive_msg) {
    last_steering = drive_msg->drive.steering_angle;
}

// The scan callback, update your occupancy grid here
void RRT_ROS_Node::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    
    // acquire the laser scan
    std::vector<float> laser_values = scan_msg->ranges;

    // set occupancy grid time stamp and reference frame
    rrt_grid.header.stamp = this->get_clock()->now();
    rrt_grid.header.frame_id = pose_to_listen;

    // reset occupancy grid
    std::fill(rrt_grid.data.begin(), rrt_grid.data.end(), 0);

    // update occupancy grid
    for (int i = 0; i < grid_height; i++)
    {
        for (int j = 0; j < grid_width; j++)
        {

            // find x, y relative to the car
            double x_cell = grid_resolution * j;
            double y_cell = grid_resolution * (i - grid_height / 2);

            // find euclidean distance
            double dist_cell = dist_x_y(x_cell, y_cell);

            // find angle
            double angle_cell = atan2(y_cell, x_cell);
            int angle_index = (angle_cell - angle_min) / angle_increment;

            // check if distance match
            if (dist_cell >= laser_values[angle_index] - safety_padding)
            {
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

/**
 * @brief Callback function for when subscribed to particle filter's inferred pose. The RRT_ROS_Node main loop happens here.
 *
 * @param pose_msg Pointer to the incoming pose message of type PoseStamped
 * @return void
 */
 // TODO NOTE FIXME (ejenny) change this to create a planner and run the planner
void RRT_ROS_Node::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
{
    // clear rrt marker array and reset marker id in a new iteration
    rrt_marks.markers.clear();
    marker_id = 0;

    // step 0: publish path and update grid origin pose
    path_msg.header.stamp = this->get_clock()->now();
    path_pub_->publish(path_msg);

    // Convert pose_msg to RRT_node
    CarState *cur_state = new CarState(0.0f, 0.0f, 0.0f, 0.0f, pose_msg->twist.twist.linear.x, last_steering);
    RCLCPP_INFO(this->get_logger(), "Cur vel %f", cur_state->get_vel());

    // Get the goal point
    Goal goal(0.0, 0.0, 0.0);
    find_goal_point(&goal);
    RCLCPP_INFO(this->get_logger(), "get goal value: %f,%f", goal.x, goal.y);

    KinematicConstraints *kin_constr = new KinematicConstraints(-3.0, 3.0, 6.0, 0.4189, 0.0);
    Planner_Params *params = new Planner_Params(goal_sample_rate, num_of_samples, grid_resolution, grid_height, grid_width, max_yaw);
    // Init Planner_RRT
    Planner_RRT planner = Planner_RRT(cur_state, &goal, &rrt_grid, NULL, kin_constr, params);

    RCLCPP_INFO(this->get_logger(), "Planner initialized");

    // Call planner plan
    vector<CarState*> path_points = planner.plan();
    RCLCPP_INFO(this->get_logger(), "Planner finished");

    for (RRT_Node *node : planner.get_nodes()) {
        if (node != NULL && node->parent != NULL)
            add_marker(node->parent->state, node->state);
    }
    RCLCPP_INFO(this->get_logger(), "Added markers");

    // for debug purpose, publish the visualization marker array
    vis_marker_array_pub_->publish(rrt_marks);

    // step 4: command steering
    // strategy: use the first steering
    RCLCPP_INFO(this->get_logger(), "found path length: %d", path_points.size());
    CarState *next_state = path_points[path_points.size() - 2];
    double steering_angle = next_state->get_alpha();
    double velocity = next_state->get_vel();
    RCLCPP_INFO(this->get_logger(), "Got steering angle %f", steering_angle);

    // command drive
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.steering_angle = steering_angle;
    drive_msg.drive.speed = velocity;

    drive_pub_->publish(drive_msg);
}


// ---------------------- end RRT_ROS_Node* methods ----------------------

void RRT_ROS_Node::log_waypoints()
{
    // use the waypoint file to fill waypoints in the path message
    // for finding the next waypoint

    // open the file
    waypoint_file.open(waypoint_file_path);

    // extract waypoints
    std::string line;
    while (std::getline(waypoint_file, line))
    {

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

void RRT_ROS_Node::find_goal_point(Goal *goal_point)
{
    // given Path msg, loop through it
    // to find the desired goal direction
    // return type is double, the angle of goal
    // point relative to current car pose

    // try to obtain current map to base_link transform
    fromFrame = waypoint_frame_id;
    toFrame = pose_to_listen;
    geometry_msgs::msg::TransformStamped curr_t;
    try
    {
        curr_t = tf_buffer_->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
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
    for (int i = start_search_index; search_count < path_length; i++)
    {
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
        bool shorter = (dist_x_y(first_transformed.pose.position.x, first_transformed.pose.position.y) <= look_ahead_dist);
        bool longer = (dist_x_y(second_transformed.pose.position.x, second_transformed.pose.position.y) > look_ahead_dist);

        // found the transform pairs for interpolation
        // - first frame is within the radius
        // - second frame is beyond the radius
        // - both first and second frames are in front of the car
        if (shorter && longer && first_transformed.pose.position.x > 0.0f && second_transformed.pose.position.x > 0.0f)
        {
            last_waypoint_index = first_frame_index;
            heading_found = true;
            break;
        }
        else
        {
            search_count++;
        }
    }

    if (heading_found)
    {
        // run interpolation to update goal location
        interpolate_points(first_transformed.pose.position.x,
                           first_transformed.pose.position.y,
                           second_transformed.pose.position.x,
                           second_transformed.pose.position.y,
                           goal_point);
    }

    return;
}

void RRT_ROS_Node::interpolate_points(double x1, double y1, double x2, double y2, Goal *goal_point)
{

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
    goal_point->x = x1 + frac * x_diff;
    goal_point->y = y1 + frac * y_diff;

    // publish that as a point, check correctness on rviz
    auto point_msg = geometry_msgs::msg::PointStamped();
    point_msg.header.stamp = this->get_clock()->now();
    point_msg.header.frame_id = pose_to_listen;
    point_msg.point.x = goal_point->x;
    point_msg.point.y = goal_point->y;
    point_msg.point.z = 0;
    point_pub_->publish(point_msg);

    return;
}

double RRT_ROS_Node::dist_x_y(double x, double y)
{

    // calculate the euclidean to (x, y)
    // assuming from (0, 0)
    // return dist

    return sqrt(x * x + y * y);
}


void RRT_ROS_Node::process_scan(std::vector<float> &scan)
{
    // process the laser scan to
    // - buffer the walls by car's length
    // - extend disparities by car's width
    // this creates a occupancy grid (configuration space)
    // for car to traverse through

    for (size_t i = 0; i < scan.size(); i++)
    {

        // buffer by car's length
        scan[i] -= safety_padding;
    }

    // (optional) TODO: smooth laser readings over a window (3 or 5)

    // extend disparities observed
}

void RRT_ROS_Node::init_grid()
{
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
    for (int i = 0; i < num_of_cells; i++)
    {
        rrt_grid.data.push_back((int8_t)0);
    }
}


/**
 * @brief For visualization
 */
void RRT_ROS_Node::add_marker(CarState *parent, CarState *child)
{
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
    new_mark.color.r = 0.0;
    new_mark.color.g = 1.0;
    new_mark.color.b = 0.0;
    new_mark.lifetime = rclcpp::Duration::from_seconds(0.1);

    // push the points in the array
    geometry_msgs::msg::Point start_point, end_point;
    start_point.x = parent->get_x();
    start_point.y = parent->get_y();
    start_point.z = 0.0f;
    end_point.x = child->get_x();
    end_point.y = child->get_y();
    end_point.z = 0.0f;
    new_mark.points.push_back(start_point);
    new_mark.points.push_back(end_point);

    // push the marker in
    rrt_marks.markers.push_back(new_mark);

    // increment marker_id
    marker_id++;

    return;
}
