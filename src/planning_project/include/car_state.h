/**
* CarState.h
*/

#ifndef CAR_STATE_H
#define CAR_STATE_H

// constants used for kinematic updates

// pi and wheel base
const double PI = 3.141592653589793;
const double WHEEL_BASE = 0.3302;

// time-step related constants (can be tuned)
const double DT = 0.05;
const int EXTEND_STEP = 10;

// opponent's velocity
const double OPPONENT_VELOCITY = 1.0;

// steering and velocity error gain
const double STEERING_GAIN = 1.0;
const double VELOCITY_GAIN = 0.5;

const double MAX_YAW = 1.5707;
const double MAX_BRAKE_ACCEL = -10.0;
const double MAX_GAS_ACCEL = 8.0;
const double MAX_VEL = 10.0;
const double MAX_STEER_ANGLE = 0.5236;
const double MAX_LATERAL_ACCEL = 7.0;
static const double TIME_FULL_STEER = 0.5;
const double MAX_STEERING_CHANGE = MAX_STEER_ANGLE * 2.0 / TIME_FULL_STEER;

struct CarState {
    double t;
    double x;
    double y;
    double yaw;
    double vel;
    double alpha; 
};

typedef struct CarState CarState;

// update 
CarState CalcNextState(CarState state, double accel, double delta_alpha);

#endif // CAR_STATE_H
