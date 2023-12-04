
#ifndef PLANNER_H
#define PLANNER_H

#include "car_state.h"
#include "position_constraints.h"
#include "velocity_constraints.h"

static const float delta_t = 0.2;

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

    within_radius(CarState *state) {
        float dx = state->x - this->x;
        float dy = state->y - this->y;
        float dist = sqrt(dx*dx + dy*dy);
        return dist <= this->r;
    }
};

class Node {
public:
    int id;
    CarState *state;
    Node *parent;
    float cost;
    float heuristic;

    Node(CarState *state, Node *parent, float cost, float heuristic) {
        this->state = state;
        this->parent = parent;
        this->cost = cost;
        this->heuristic = heuristic;
    }

    float get_total_cost() {
        return this->cost + this->heuristic;
    }
};

void plan(CarState *init_state, Goal *goal, PathConstraints *path_constr, DynamicObstacleConstraint *dyn_constr, KinematicConstraint *kin_constr, CarState *final_state);

void run_planner();

#endif // PLANNER_H
