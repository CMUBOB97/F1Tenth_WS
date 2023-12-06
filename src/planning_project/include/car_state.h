/**
* CarState.h
*/

#ifndef CAR_STATE_H
#define CAR_STATE_H

const int STATE_SIZE = 6;

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

class CarState {
private:
    double state[STATE_SIZE];

public:
    CarState(double t, double x, double y, double yaw, double vel, double alpha) {
        state[0] = t;
        state[1] = x;
        state[2] = y;
        state[3] = yaw;
        state[4] = vel;
        state[5] = alpha;
    }

    double get_t() const { return state[0]; }
    double get_x() const { return state[1]; }
    double get_y() const { return state[2]; }
    double get_yaw() const { return state[3]; }
    double get_vel() const { return state[4]; }
    double get_alpha() const { return state[5]; }
    double *get_state() { return state; }
    double *get_state_no_t() { return state + 1; }
};

// update 
CarState *CalcNextState(CarState *state, double accel, double delta_alpha);

#endif // CAR_STATE_H
