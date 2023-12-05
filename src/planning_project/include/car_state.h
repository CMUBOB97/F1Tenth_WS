/**
* CarState.h
*/

#ifndef CAR_STATE_H
#define CAR_STATE_H

// constants used for kinematic updates

// pi and wheel base
const double pi = 3.141592653589793;
const double wheel_base = 0.3302;

// time-step related constants (can be tuned)
const double dt = 0.05;
const int extend_step = 10;

// opponent's velocity
const double opponent_velocity = 1.0;

// steering and velocity error gain
const double steering_gain = 1.0;
const double velocity_gain = 1.0;

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
