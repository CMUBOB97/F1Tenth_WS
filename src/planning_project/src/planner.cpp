/**
 *
 */

#include "planner.h"
#include "rrt.h"
#include <iostream>

static const int TRAPPED = 0;
static const int ADVANCED = 1;
static const int REACHED = 2;

std::vector<CarState*> Planner_RRT::plan()
{
    std::cout << "Planning..." << std::endl;
    RRT_Node *node_to_extend;
    for (int i = 0; i < params->num_samples;)
    {
        std::vector<double> random_point = sample_config_space();
        node_to_extend = nearest_in_config(random_point);

        int expand_status = extend(node_to_extend);
        if (expand_status == REACHED)
        {
            std::cout << "Reached goal!" << std::endl;
            break;
        }
        else if (expand_status == ADVANCED)
        {
            i++;
        }
    }

    std::vector<CarState*> path = find_path(node_to_extend);
    return path;
}

RRT_Node *Planner_RRT::nearest_in_config(std::vector<double> &sample)
{
    RRT_Node *best_node = NULL;
    double best_dist = -1.0f;

    for (auto node : this->nodes) {
        double x_diff = std::abs(node->state->get_x() - sample[0]);
        double y_diff = std::abs(node->state->get_y() - sample[1]);
        double yaw_diff = std::abs(node->state->get_yaw() - sample[2]);
        double vel_diff = std::abs(node->state->get_vel() - sample[3]);
        double x_diff_norm = x_diff / params->grid_height;
        double y_diff_norm = y_diff / params->grid_width * 2;
        double yaw_diff_norm = yaw_diff / params->max_yaw * 2;
        double vel_diff_norm = vel_diff / kin_constr->max_vel;
        double dist = std::sqrt(x_diff_norm * x_diff_norm + y_diff_norm * y_diff_norm +
                                yaw_diff_norm * yaw_diff_norm + vel_diff_norm * vel_diff_norm);

        // update
        if (best_dist < 0 || dist < best_dist)
        {
            best_dist = dist;
            best_node = node;
        }

    }

    double *sample_array = &sample[0];
    kd_node *nearest_kd = tree->nearest(sample_array); 
    RRT_Node *nearest_new = nodes[nearest_kd->id]; // TODO check
    // assert(nearest_new == best_node);
    return best_node;
}


/**
 * @brief The function extend attempts to add a new node between nearest node and sampled point based on step size.
 * 
 * @param tree The tree to which the node is to be added.
 * @param nearest The nearest existing node from the sampled point.
 * @param sample The sampled point.
 * @return Status indicating the result of the operation:
 *         - TRAPPED: New node will not be added to the tree (hit an obstacle).
 *         - ADVANCED: New node is added to the tree, but goal is not reached yet.
 *         - REACHED: New node is added to the tree, and goal is reached.
 */
int Planner_RRT::extend(RRT_Node *parent_node)
{
    // check collision variable
    bool collision_status = false;
    bool goal_reached = false;

    std::tuple<double, double> actions = sample_actions();
    double d_alpha = std::get<0>(actions);
    double accel = std::get<1>(actions);

    // Construct current car state
    CarState *state_sim = parent_node->state;
    CarState *state_sim_next;

    // forward simulate towards the goal state
    for (int i = 0; i < extend_step; i++)
    {
        state_sim_next = CalcNextState(state_sim, accel, d_alpha);

        collision_status = check_collision(state_sim, state_sim_next);
        if (collision_status == true)
        {
            return TRAPPED;
        }
        // TODO (ejenny): do dynamic collision check here

        std::cout << "state_sim_next: " << state_sim_next->get_x() << ", " << state_sim_next->get_y();
        std::cout << ", " << state_sim_next->get_yaw() << ", " << state_sim_next->get_vel() << ", " << state_sim_next->get_alpha() << std::endl;
        std::cout << "goal: " << goal->x << ", " << goal->y << "radius: " << goal->r << std::endl;
        goal_reached = goal->at_goal(state_sim_next);
        if (goal_reached)
        {
            break;
        }

        state_sim = state_sim_next;
    }
    RRT_Node *new_node = add_node(parent_node, state_sim_next);

    if (goal_reached)
    {
        return REACHED;
    }
    else
    {
        return ADVANCED;
    }
}

RRT_Node* Planner_RRT::add_node(RRT_Node *parent, CarState *new_state)
{
    int id = nodes.size();
    RRT_Node *new_node = new RRT_Node(new_state, parent, nodes.size());
    nodes.push_back(new_node);
    tree->insert(new_node->state->get_state_no_t(), id);
    return new_node;
}



/**
 * @brief This method returns a boolean indicating if the path between the nearest node and the new node created from steering is collision free.
 * 
 * @param nearest_node Nearest node on the tree to the sampled point.
 * @param new_node New node created from steering.
 * @return bool True if in collision, false otherwise.
 */
bool Planner_RRT::check_collision(CarState *new_state, CarState *prev_state)
{
    // find nearest and new nodes' indices in occupancy grid
    double prev_grid_x = prev_state->get_x() / params->grid_resolution;
    double prev_grid_y = prev_state->get_y() / params->grid_resolution + params->grid_height / 2;
    double new_grid_x = new_state->get_x() / params->grid_resolution;
    double new_grid_y = new_state->get_y() / params->grid_resolution + params->grid_height / 2;

    int start_x = floor((prev_grid_x < new_grid_x) ? prev_grid_x : new_grid_x);
    int end_x = ceil((prev_grid_x < new_grid_x) ? new_grid_x : prev_grid_x);
    int start_y = floor((prev_grid_y < new_grid_y) ? prev_grid_y : new_grid_y);
    int end_y = ceil((prev_grid_y < new_grid_y) ? new_grid_y : prev_grid_y);

    // draw a square on occupancy grid between these two points
    // check for occupancy within this box
    for (int i = start_y; i < end_y; i++)
    {
        for (int j = start_x; j < end_x; j++)
        {
            int index = i * params->grid_width + j;
            if (rrt_grid->data[index] > 50)
            {
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief This method traverses the tree from the node that has been determined as goal.
 * 
 * @param latest_added_node Latest addition to the tree that has been determined to be close enough to the goal.
 * @return std::vector<RRT_Node> The vector that represents the order of the nodes traversed as the found path.
 */
std::vector<CarState*> Planner_RRT::find_path(RRT_Node *leaf)
{
    std::vector<CarState*> path;

    RRT_Node *search_node = leaf;

    path.push_back(search_node->state);

    while (search_node->parent != NULL)
    {
        search_node = search_node->parent;
        path.push_back(search_node->state);
    }

    return path;
}

/**
 * @brief This method returns a sampled point from the free space. 
 * You should restrict so that it only samples a small region of interest 
 * around the car's current position.
 * 
 * @param goal Goal location x, y for biased sampling.
 * @param goal_status Argument for if x, y pair is a valid goal point.
 * @return std::vector<double> The sampled point in free space.
 */
std::vector<double> Planner_RRT::sample_config_space()
{
    double rand_x, rand_y, rand_yaw, rand_vel, rand_alpha;

    // Sampling towards the goal
    double goal_sample = sample_type(gen);

    // determine random x, y, yaw, velocity using goal sample rate
    if (goal_sample <= params->goal_sample_chance)
    {
        rand_x = this->goal->x;
        rand_y = this->goal->y;
        rand_yaw = atan2(this->goal->y, this->goal->x);
        rand_vel = kin_constr->max_vel * vel_gen(gen);
        rand_alpha = 0.0; //TODO: this is not correct
    }
    else
    {
        rand_x = params->grid_resolution * params->grid_width * x_dist(gen);
        rand_y = params->grid_resolution * params->grid_height * (y_dist(gen) - 0.5f);
        rand_yaw = params->max_yaw * (yaw_gen(gen) - 0.5f);
        rand_vel = kin_constr->max_vel * vel_gen(gen);
        rand_alpha = kin_constr->max_alpha * (alpha_gen(gen) - 0.5f);
    }

    std::vector<double> sampled_point = {rand_x, rand_y, rand_yaw, rand_vel, rand_alpha};

    return sampled_point;
}

/**
 * @brief Change in steering angle, acceleration limited to dynamic model constraints.
 */
std::tuple<double, double> Planner_RRT::sample_actions()
{
    double rand_delta_alpha = 0; // TODO randomly sample
    double rand_accel = 0;   

    // Enforce kinematic constraints
    if (rand_accel < kin_constr->max_brake_accel)
    {
        rand_accel = kin_constr->max_brake_accel;
    }
    else if (rand_accel > kin_constr->max_gas_accel)
    {
        rand_accel = kin_constr->max_gas_accel;
    }
    if (rand_delta_alpha < -kin_constr->max_steer_change)
    {
        rand_delta_alpha = -kin_constr->max_steer_change;
    }
    else if (rand_delta_alpha > kin_constr->max_steer_change)
    {
        rand_delta_alpha = kin_constr->max_steer_change;
    }
    return std::make_tuple(rand_delta_alpha, rand_accel);
}


// ---------------------- begin RRT* methods ----------------------

/**
 * @brief This method returns the set of Nodes in the neighborhood of a node.
 * 
 * @param tree The current tree.
 * @param node The node to fnearest_in_configind the neighborhood for.
 * @return std::vector<int> The index of the nodes in the neighborhood.
 */
std::vector<RRT_Node*> Planner_RRT::near(RRT_Node *node)
{

    std::vector<RRT_Node*> neighborhood;
    // TODO use kd_tree

    return neighborhood;
}

/*
int rewire(double *sample, double radius, map_struct env)
{
    int nearest = find_nearest_node(sample);
    const double *x_near = get_config(nearest);

    double *new_config = new double[num_dofs];
    if (_extend(x_near, sample, new_config, radius, env) == -1)
    {
        delete[] new_config;
        return -1;
    }

    std::vector<std::pair<int, double>> neighbors = kd_tree->radius_search(new_config, radius); // Index, distance
    double dist_to_best = distance(new_config, nodes[nearest]->config, num_dofs);
    double lowest_cost = nodes[nearest]->cost + dist_to_best;
    dassert(lowest_cost > 0);
    int best_parent = nearest;

    std::vector<std::pair<int, double>> connectable = {};
    double dist = 0;
    int i = 0;
    // Find the best parent in the neighborhood
    for (auto n : neighbors)
    {
        i = n.first;
        dist = n.second;
        if (!can_connect(new_config, nodes[i]->config, dist, num_dofs, env))
            continue;

        connectable.push_back(n);
        if (nodes[i]->cost + dist < lowest_cost)
        {
            lowest_cost = nodes[i]->cost + dist;
            dist_to_best = dist;
            best_parent = i;
            dassert(lowest_cost > 0);
        }
    }
    int new_node = add_node(new_config, best_parent, dist_to_best);
    if (best_parent != nearest)
    {
        DEBUG(printf(" - Foud better connection, lowest cost: %.2f, distance to lowest cost: %.2f\n", lowest_cost, dist_to_best));
    }
    TRACE(printf("New node: %d, cost %f\n", new_node, nodes[new_node]->cost));

    node *neighbor = NULL;

    double delta = 0;
    std::set<int> swapped_parents = {};
    // Update any neighbors if we can improve their cost
    for (auto n : connectable)
    {
        neighbor = nodes[n.first];
        delta = n.second + lowest_cost - neighbor->cost;
        if (delta < 0)
        {
            swap_parent(n.first, new_node);
            swapped_parents.insert(n.first);
            TRACE(printf("   Neighbor %d, old %.2f, new cost %.2f, children: ", n.first, neighbor->cost, lowest_cost + n.second));
            TRACE(print_children(n.first));
            neighbor->cost = lowest_cost + n.second;
            for (int child : neighbor->children)
            {
                children_to_update.push(std::make_pair(child, delta));
                dassert(neighbor->cost < nodes[child]->cost);
            }
        }
    }

    // Update costs of any children
    int update_count = 0;
    int id = 0;
    while (!children_to_update.empty())
    {
        update_count++;
        id = children_to_update.front().first;
        if (swapped_parents.count(id) == 1)
        {
            children_to_update.pop();
            continue;
        }

        neighbor = nodes[id];
        delta = children_to_update.front().second;
        children_to_update.pop();
        neighbor->cost += delta;
        TRACE(printf("  Updated node %d has %lu children: ", id, neighbor->children.size()));
        TRACE(print_children(id));
        dassert(neighbor->cost > nodes[neighbor->parent]->cost);
        for (int child : neighbor->children)
        {
            children_to_update.push(std::make_pair(child, delta));
            dassert(neighbor->cost < nodes[child]->cost);
        }
    }
    if (update_count > 0)
        DEBUG(printf("  Updated %d nodes\n", update_count));

    return new_node;
}
*/
