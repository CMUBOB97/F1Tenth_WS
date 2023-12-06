
#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <vector>
#include <random>
#include <string>

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "car_state.h"
#include "position_constraints.h"
#include "kinematic_constraints.h"
#include "kd_tree.h"
// #include "goal.h"

class Goal {
public:
    float x;
    float y;
    float r;

    Goal(float x, float y, float r) {
        this->x = x;
        this->y = y;
        this->r = r;
    }

    bool at_goal(CarState *state) {
        float dx = state->get_x() - this->x;
        float dy = state->get_y() - this->y;
        float dist = sqrt(dx*dx + dy*dy);
        return dist <= this->r;
    }
};

static const float delta_t = 0.2;

class RRT_Node {
public:
    int id;
    CarState *state;
    RRT_Node *parent;
    double heuristic;

    RRT_Node(CarState *state, RRT_Node *parent, int id,     double heuristic=0) {
        this->state = state;
        this->parent = parent;
        this->heuristic = heuristic;
        this->id = id;
    }

    double get_total_cost() {
        return this->state->get_t() + this->heuristic;
    }

    // TODO(ejenny): tweak to use all states
    static double  distance(RRT_Node *n1, RRT_Node *n2) {
        float dx = n1->state->get_x() - n2->state->get_x();
        float dy = n1->state->get_y() - n2->state->get_y();
        return sqrt(dx*dx + dy*dy);
    }
};

class Planner_Params {
public:
    double goal_sample_chance;
    int num_samples;
    double grid_resolution;
    double grid_width;
    double grid_height;
    double max_yaw;
    int marker_id;


    Planner_Params(double goal_sample_chance, int num_samples, double grid_resolution, double grid_width, double grid_height, double max_yaw) {
        this->goal_sample_chance = goal_sample_chance;
        this->num_samples = num_samples;
        this->grid_resolution = grid_resolution;
        this->grid_width = grid_width;
        this->grid_height = grid_height;
        this->max_yaw = max_yaw;
    }
};

class Planner_RRT {
private:
    std::vector<RRT_Node *> nodes;
    KDTree *tree;
    Goal *goal;
    nav_msgs::msg::OccupancyGrid *rrt_grid;
    DynamicObstacleConstraint *dyn_constr;
    KinematicConstraints *kin_constr;
    visualization_msgs::msg::MarkerArray *marker_array;
    Planner_Params *params;

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> sample_type;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;
    std::uniform_real_distribution<> yaw_gen;
    std::uniform_real_distribution<> vel_gen;
    std::uniform_real_distribution<> alpha_gen;

    std::vector<double> sample_config_space();
    std::tuple<double, double> sample_actions();
    RRT_Node *nearest_in_config(std::vector<double> &sample);
    RRT_Node *nearest_in_conig(std::vector<double> &sample);
    RRT_Node *add_node(RRT_Node *parent, CarState *new_state);
    bool check_collision(CarState *new_state, CarState *prev_state);
    std::vector<CarState*> find_path(RRT_Node *leaf);
    int extend(RRT_Node *parent_node);
    std::vector<RRT_Node*> near(RRT_Node *node);

public:
    Planner_RRT (CarState *start, Goal *goal, nav_msgs::msg::OccupancyGrid *rrt_grid, DynamicObstacleConstraint *dyn_constr, KinematicConstraints *kin_constr, Planner_Params *params) {
        this->goal = goal;
        this->rrt_grid = rrt_grid;
        this->dyn_constr = dyn_constr;
        this->kin_constr = kin_constr;
        this->params = params;
        this->tree = new KDTree(STATE_SIZE, start->get_state());
        this->add_node(NULL, start);
    }

    std::vector<CarState*> plan();
    std::vector<RRT_Node*> get_nodes() { return this->nodes; }
};

#endif // PLANNER_H
