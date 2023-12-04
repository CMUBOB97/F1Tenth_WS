#include <cmath>
#include "car_state.h"

// next car state kinematic update
CarState CalcNextState(CarState state, float accel, float delta_alpha) {

    // next car state
    CarState next;

    // update time
    next.t = state.t + dt;

    // TODO(chig): update x
    next.x = state.x + dt * cos(state.yaw) * state.vel;

    // TODO(chig): update y
    next.y = state.y + dt * sin(state.yaw) * state.vel;
    
    // TODO(chig): update yaw
    next.yaw = state.yaw + dt * state.vel * tan(state.alpha) / wheel_base;

    // TODO(chig): update vel
    next.vel = state.vel + dt * accel;
    
    // TODO(chig): update steering
    next.alpha = state.alpha + dt * delta_alpha;

    return next;

}