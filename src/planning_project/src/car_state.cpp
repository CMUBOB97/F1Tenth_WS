#include <cmath>
#include "car_state.h"

// next car state kinematic update
CarState *CalcNextState(CarState *state, double accel, double delta_alpha)
{
    float next_t = state->get_t() + dt;
    float next_x = state->get_x() + dt * cos(state->get_yaw() + delta_alpha) * state->get_vel();
    float next_y = state->get_y() + dt * sin(state->get_yaw() + delta_alpha) * state->get_vel();
    float next_yaw = state->get_yaw() + dt * state->get_vel() * tan(state->get_alpha()) / wheel_base;

    // TODO(chig): update vel
    float next_vel = state->get_vel() + dt * accel;
    float next_alpha = state->get_alpha() + dt * delta_alpha;

    CarState *next = new CarState(next_t, next_x, next_y, next_yaw, next_vel, next_alpha);

    return next;
}