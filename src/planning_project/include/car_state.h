/**
* CarState.h
*/

#ifndef CAR_STATE_H
#define CAR_STATE_H

#include <random>

// constants used for kinematic updates

// pi and wheel base
const double WHEEL_BASE = 0.3302;

// time-step related constants (can be tuned)
const double DT = 0.001;
const int EXTEND_STEP = 100;

// opponent's velocity
const double OPPONENT_VELOCITY = 1.0;

// steering and velocity error gain
const double STEERING_GAIN = 1.0;
const double VELOCITY_GAIN = 0.5;

const double MAX_BRAKE_ACCEL = -10.0;
const double MAX_GAS_ACCEL = 8.0;
const double MAX_LATERAL_ACCEL = 7.0;
const double MAX_STEER_ANGLE = 0.5236;
static const double TIME_FULL_STEER = 0.5;
const double MAX_STEERING_CHANGE = MAX_STEER_ANGLE * 2.0 / TIME_FULL_STEER;

const double PROBABILITY_STRAIGHT = 0.3;
const double PROBABILITY_COAST = 0.3;

const double AT_GOAL_RADIUS = 0.1; // meters

extern double MAX_YAW; // 1.5707
extern double MAX_VEL; // 10.0

extern double GRID_RESOLUTION;
extern int GRID_WIDTH;
extern int GRID_HEIGHT; 

extern double GOAL_SAMPLE_RATE;
extern double LOOK_AHEAD_DIST;

struct CarState_t {
    double t;
    double x;
    double y;
    double yaw;
    double vel;
    double alpha; 
    double accel;
    double delta_alpha;
    int num_extensions;
};

typedef struct CarState_t CarState;

void print_car_state(CarState &state);
CarState CalcNextState(CarState &state, double accel, double delta_alpha);
bool is_kinematically_feasible(CarState &state);  

double euclidean_dist(double x, double y);
double euclidean_dist(std::vector<double> &v1, std::vector<double> &v2);

double car_state_distance(CarState &a, CarState &b);
double car_state_distance(CarState &a, std::vector<double> &b);

std::pair<double, double> sample_actions(void);
std::pair<double, double> sample_actions_extreme(void);
std::vector<double> sample_config(std::vector<double> &goal, bool goal_status);


bool is_goal(CarState &state, std::vector<double> &goal_point, bool goal_is_reachable); // TODO move?

#endif // CAR_STATE_H
