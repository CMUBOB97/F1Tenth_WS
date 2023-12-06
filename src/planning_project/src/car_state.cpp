#include <cmath>
#include "car_state.h"

// next car state kinematic update
CarState CalcNextState(CarState state, double accel, double delta_alpha) {

    // next car state
    CarState next;

    // update time
    next.t = state.t + DT;
    next.x = state.x + DT * cos(state.yaw + delta_alpha) * state.vel;
    next.y = state.y + DT * sin(state.yaw + delta_alpha) * state.vel;
    next.yaw = state.yaw + DT * state.vel * tan(state.alpha) / WHEEL_BASE;
    next.vel = state.vel + DT * accel;
    next.alpha = state.alpha + delta_alpha;

    if (next.alpha > MAX_STEER_ANGLE) {
        next.alpha = MAX_STEER_ANGLE;
    } else if (next.alpha < -MAX_STEER_ANGLE) {
        next.alpha = -MAX_STEER_ANGLE;
    }
    if (next.vel > MAX_VEL) {
        next.vel = MAX_VEL;
    } else if (next.vel < 0.0) {
        next.vel = 0.0;
    }

    double radius = WHEEL_BASE / tan(next.alpha);
    double max_val = sqrt(MAX_LATERAL_ACCEL * abs(radius));
    if (next.vel > max_val) {
        next.vel = max_val;
    }

    return next;
}