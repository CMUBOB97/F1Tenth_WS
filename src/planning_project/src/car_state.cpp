#include <cmath>
#include "car_state.h"

// next car state kinematic update
CarState CalcNextState(CarState state, double accel, double delta_alpha)
{
    // next car state
    CarState next;

    // update time
    next.t = state.t + dt;
    next.x = state.x + dt * cos(state.yaw + delta_alpha) * state.vel;
    next.y = state.y + dt * sin(state.yaw + delta_alpha) * state.vel;
    next.yaw = state.yaw + dt * state.vel * tan(state.alpha) / wheel_base;
    if (next.yaw < 0) {
        next.yaw += 2 * pi;
    }
    // TODO(chig): update vel
    next.vel = state.vel + dt * accel;
    next.alpha = state.alpha + dt * delta_alpha;

    return next;
}