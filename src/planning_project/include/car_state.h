/**
* CarState.h
*/

#ifndef CAR_STATE_H
#define CAR_STATE_H

#include <random>

std::mt19937 generator;
std::uniform_real_distribution<double> uniform_sampler(0.0, 1.0);
std::normal_distribution<double> normal_sampler(0.0, 1.0);

// constants used for kinematic updates

// pi and wheel base
const double WHEEL_BASE = 0.3302;

// time-step related constants (can be tuned)
const double DT = 0.05;
const int EXTEND_STEP = 5;

// opponent's velocity
const double OPPONENT_VELOCITY = 1.0;

// steering and velocity error gain
const double STEERING_GAIN = 1.0;
const double VELOCITY_GAIN = 0.5;

extern const double MAX_YAW; // 1.5707
extern const double MAX_VEL; // 10.0

const double MAX_BRAKE_ACCEL = -10.0;
const double MAX_GAS_ACCEL = 8.0;
const double MAX_LATERAL_ACCEL = 7.0;
const double MAX_STEER_ANGLE = 0.5236;
static const double TIME_FULL_STEER = 0.5;
const double MAX_STEERING_CHANGE = MAX_STEER_ANGLE * 2.0 / TIME_FULL_STEER;

const double PROBABILITY_STRAIGHT = 0.3;
const double PROBABILITY_COAST = 0.3;

extern const double GRID_RESOLUTION;
extern const int GRID_WIDTH;
extern const int GRID_HEIGHT; 

extern const double GOAL_SAMPLE_RATE;

struct CarState_t {
    double t;
    double x;
    double y;
    double yaw;
    double vel;
    double alpha; 
    double accel;
    double delta_alpha;
};

typedef struct CarState_t CarState;

CarState CalcNextState(CarState &state, double accel, double delta_alpha);
bool is_kinematically_feasible(CarState &state);  

double euclidean_dist(double x, double y);
double euclidean_dist(std::vector<double> &v1, std::vector<double> &v2);

double car_state_distance(CarState &a, CarState &b);
double car_state_distance(CarState &a, std::vector<double> &b);

std::pair<double, double> sample_actions(void);
std::pair<double, double> sample_actions_extreme(void);
std::vector<double> sample_config(std::vector<double> &goal, bool goal_status);

#endif // CAR_STATE_H
