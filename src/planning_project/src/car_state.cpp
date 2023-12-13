#include <cmath>
#include <vector>
#include <tuple>
#include <iostream>

#include "car_state.h"

static std::mt19937 generator;
static std::uniform_real_distribution<double> uniform_sampler(0.0, 1.0);
static std::normal_distribution<double> normal_sampler(0.0, 1.0);
static std::uniform_real_distribution<double> accel_sampler(MAX_BRAKE_ACCEL, MAX_GAS_ACCEL);


double MAX_YAW = 0.0; // 1.5707
double MAX_VEL = 0.0; // 10.0
double GRID_RESOLUTION = 0.0;
int GRID_WIDTH = 0;
int GRID_HEIGHT = 0; 
double GOAL_SAMPLE_RATE = 0.0;
double LOOK_AHEAD_DIST = 0.0;

/**
 * Prints on one line
 */
void print_car_state(CarState &state)
{
    std::cout << " State t: " << state.t << ", x: " << state.x << ", y: " << state.y << ", yaw: " << state.yaw << ", vel: " << state.vel << ", alpha: " << state.alpha << std::endl;
}

// next car state kinematic update
CarState CalcNextState(CarState &state, double accel, double delta_alpha)
{
    // next car state
    CarState next;

    // update time
    next.t = state.t + DT;
    double next_vel = state.vel + DT * accel;
    next.x = state.x + DT * cos(state.yaw + delta_alpha) * next_vel;
    next.y = state.y + DT * sin(state.yaw + delta_alpha) * next_vel;
    next.yaw = state.yaw + DT * next_vel * tan(state.alpha) / WHEEL_BASE;
    next.alpha = state.alpha + delta_alpha;
    next.vel = next_vel;

    next.accel = accel;
    next.delta_alpha = delta_alpha; 
    next.num_extensions = state.num_extensions + 1;

    // (ejenny) Redudant limits

    if (next.alpha > MAX_STEER_ANGLE)
    {
        next.alpha = MAX_STEER_ANGLE;
    }
    else if (next.alpha < -MAX_STEER_ANGLE)
    {
        next.alpha = -MAX_STEER_ANGLE;
    }
    if (next.vel > MAX_VEL)
    {
        next.vel = MAX_VEL;
    }
    else if (next.vel < 0.0)
    {
        next.vel = 0.0;
    }

    if (fabs(next.alpha) < 1e-5) {
        return next;
    }

    double radius = WHEEL_BASE / tan(next.alpha);
    double max_val = 0.99 * std::sqrt(MAX_LATERAL_ACCEL * std::fabs(radius));
    if (next.vel > max_val)
    {
        next.vel = max_val;
    }

    // std::cout << "accel: " << accel << ", delta_alpha: " << delta_alpha;
    // std::cout << ", next_vel: " << next_vel << ", state.alpha: " << state.alpha;
    // std::cout << ", max_val: " << max_val << ", radius: " << radius;
    // std::cout << ", MAX_VEL: " << MAX_VEL << std::endl;

    return next;
}

double car_state_distance(CarState &a, CarState &b)
{
    double x_diff = std::fabs(a.x - b.x);
    double y_diff = std::fabs(a.y - b.y);
    double yaw_diff = std::fabs(a.yaw - b.yaw);
    double vel_diff = std::fabs(a.vel - b.vel);
    double alpha_diff = std::fabs(a.alpha - b.alpha);
    double x_diff_norm = x_diff / (GRID_WIDTH * GRID_RESOLUTION);
    double y_diff_norm = y_diff / (GRID_HEIGHT * GRID_RESOLUTION);
    double yaw_diff_norm = yaw_diff / MAX_YAW * 2;
    double vel_diff_norm = vel_diff / MAX_VEL;
    double alpha_diff_norm = alpha_diff / MAX_STEER_ANGLE * 2;
    double dist = std::sqrt(
            x_diff_norm * x_diff_norm 
            + y_diff_norm * y_diff_norm
            + yaw_diff_norm * yaw_diff_norm 
            + vel_diff_norm * vel_diff_norm 
            + alpha_diff_norm * alpha_diff_norm * 0.0);
    return dist;
}

double car_state_distance(CarState &a, std::vector<double> &b)
{
    CarState b_state;
    b_state.x = b[0];
    b_state.y = b[1];
    b_state.yaw = b[2];
    b_state.vel = b[3];
    b_state.alpha = b[4];
    return car_state_distance(a, b_state);
}

static std::pair<double, double> limit_actions(double accel, double delta_alpha)
{
    if (accel > MAX_GAS_ACCEL)
    {
        accel = MAX_GAS_ACCEL;
    }
    else if (accel < MAX_BRAKE_ACCEL)
    {
        accel = MAX_BRAKE_ACCEL;
    }
    if (delta_alpha > MAX_STEERING_CHANGE)
    {
        delta_alpha = MAX_STEERING_CHANGE;
    }
    else if (delta_alpha < -MAX_STEERING_CHANGE)
    {
        delta_alpha = -MAX_STEERING_CHANGE;
    }
    return std::make_pair(accel, delta_alpha);
}

std::pair<double, double> sample_actions_extreme(void)
{
    double accel = 0.0;
    double delta_alpha = 0.0;
    double steering_sample = uniform_sampler(generator);
    double accel_sample = uniform_sampler(generator);
    if (steering_sample > PROBABILITY_STRAIGHT)
    {
        delta_alpha = steering_sample > 0.5 * (1.0 - PROBABILITY_STRAIGHT) + PROBABILITY_STRAIGHT ? MAX_STEERING_CHANGE * DT : -MAX_STEERING_CHANGE * DT;
    }
    if (accel_sample > PROBABILITY_COAST)
    {
        accel = accel_sample > 0.5 * (1.0 - PROBABILITY_COAST) + PROBABILITY_COAST ? MAX_GAS_ACCEL : MAX_BRAKE_ACCEL;
    }
    // TODO add some gaussian noise to accel and delta_alpha
    return limit_actions(accel, delta_alpha);
}

std::pair<double, double> sample_actions()
{
    double rand_delta_alpha = (uniform_sampler(generator) - 0.5f) * 2.0 * MAX_STEERING_CHANGE * DT; // TODO what is this 4.0?
    double accel_diff = MAX_GAS_ACCEL - MAX_BRAKE_ACCEL;
    double accel_sample = uniform_sampler(generator);
    double rand_accel = accel_diff * accel_sample + MAX_BRAKE_ACCEL;
    // std::cout << "Sample: " << accel_sample << ", accel: " << rand_accel << std::endl;

    return limit_actions(rand_accel, rand_delta_alpha);
}

std::vector<double> sample_config(std::vector<double> &goal, bool goal_status)
{
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    //     goal x and y: for biased sampling, x and y for goal location
    //     goal_status: argument for if x, y pair is a valid goal point
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    // random x, y, yaw, velocity
    double rand_x, rand_y, rand_yaw, rand_vel, rand_alpha;

    // first, sample to determine if we want to sample the goal
    // if the goal pair is not valid, don't let it be sampled
    double goal_sample = (goal_status) ? uniform_sampler(generator) : 1.0f;

    // determine random x, y, yaw, velocity using goal sample rate
    // TODO make goal samples slightly random still
    if (goal_sample <= GOAL_SAMPLE_RATE)
    {
        rand_x = goal[0];
        rand_y = goal[1];
        rand_yaw = atan2(goal[1], goal[0]);
        rand_vel = euclidean_dist(goal[0], goal[1]) / (DT * EXTEND_STEP);
        rand_alpha = 0.0;
    }
    else
    {
        rand_x = GRID_RESOLUTION * GRID_WIDTH * uniform_sampler(generator);
        rand_y = GRID_RESOLUTION * GRID_HEIGHT * (uniform_sampler(generator) - 0.5f);
        rand_yaw = MAX_YAW * (uniform_sampler(generator) - 0.5f) * 2;
        rand_vel = MAX_VEL * uniform_sampler(generator);
        rand_alpha = (uniform_sampler(generator) - 0.5f) * 2 * MAX_STEER_ANGLE;
    }

    std::vector<double> sampled_point = {rand_x, rand_y, rand_yaw, rand_vel};

    return sampled_point;
}

double euclidean_dist(double x, double y)
{
    // calculate the euclidean distance to (x, y)
    // assuming from (0, 0)
    // return dist

    return sqrt(x * x + y * y);
}

double euclidean_dist(std::vector<double> &v1, std::vector<double> &v2)
{
    // calculate the euclidean distance between two nodes
    // return dist
    double total = 0;
    for (int i = 0; i < v1.size(); i++)
    {
        total += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    }
    double dist = sqrt(total);
    return dist;
}

bool is_kinematically_feasible(CarState &state) {
    // This method returns a boolean indicating if the state is kinematically
    // feasible
    // Args:
    //    state (CarState): the state to check
    // Returns:
    //    feasible (bool): true if feasible, false otherwise

    if (state.vel > MAX_VEL || state.vel < 0) {
        std::cout << "Velocity Failure: " << state.vel << std::endl;
        return false;
    }
    if (state.alpha > MAX_STEER_ANGLE || state.alpha < -MAX_STEER_ANGLE) {
        std::cout << "Steering angle Failure: " << state.alpha << std::endl;
        return false;
    }

    // Check lateral accel (avoid divide by zero if tan(alpha) == 0
    if (fabs(state.alpha) < 1e-5) {
        return true;
    }

    double radius = WHEEL_BASE / tan(state.alpha);
    double accel = state.vel * state.vel / fabs(radius);
    if (accel > MAX_LATERAL_ACCEL) {
        std::cout << "Lateral accel Failure: " << accel << ", vel: " << state.vel << ", radius: " << radius << std::endl;
        return false;
    }

    return true;
}


bool is_goal(CarState &state, std::vector<double> &goal, bool goal_is_reachable) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal (double vector): vector of goal point {x, y}
    //   goal_is_reachable (bool): if x and y are not goals, just check for how far latest_added_node has expanded
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    
    // the case where goal is valid
    if (goal_is_reachable) {
        double x_diff = state.x - goal[0];
        double y_diff = state.y - goal[1];
        if (euclidean_dist(x_diff, y_diff) <= AT_GOAL_RADIUS) {
            close_enough = true;
        }
    } else { // when goal is invalid, check for distance expanded
        double expand_dist = euclidean_dist(state.x, state.y);
        // TODO: if we need to check yaw and velocity difference, come back later
        if (expand_dist > LOOK_AHEAD_DIST) {
            close_enough = true;
        }
    }
    // if (close_enough)
    // {
    //     std::cout << "GOAL FOUND!, reachable: " << goal_is_reachable;
    //     print_car_state(state);
    // }

    return close_enough;
}

