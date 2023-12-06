/**
* kinematic_constraints.h
*/


#ifndef KINEMATIC_CONSTRAINTS_H
#define KINEMATIC_CONSTRAINTS_H

#include "math.h"

static const double TRACK_WIDTH = 1.5; // TODO hardcoded
static const double WHEEL_BASE = 2.5; // TODO hardcoded

class KinematicConstraints {
public:
    double max_brake_accel;
    double max_gas_accel;
    double max_vel;
    double max_alpha;
    double max_steer_change;
    double max_lateral_accel;

    KinematicConstraints(double max_brake_accel, double max_gas_accel, double max_vel, double max_steer_angle, double max_lateral_accel) {
        this->max_brake_accel = max_brake_accel;
        this->max_gas_accel = max_gas_accel;
        this->max_vel = max_vel;
        this->max_alpha = max_steer_angle;
        this->max_lateral_accel = max_lateral_accel;
        max_steer_change = 0.1; // TODO hardcoded
    }

    bool IsStateWithinConstraints(CarState *state) {
        if (state->get_vel() > max_vel || state->get_vel() < 0) {
            return false;
        }
        if (state->get_alpha() > max_alpha || state->get_alpha() < -max_alpha) {
            return false;
        }
        
        // Check lateral accel (avoid divide by zero)
        if (state->get_alpha() == 0) {
            return true;
        }
        double radius = WHEEL_BASE / tan(state->get_alpha());
        double accel = state->get_vel() * state->get_vel() / abs(radius);
        if (accel > max_lateral_accel) {
            return false;
        }
        
        return true;
    }
};

#endif // KINEMATIC_CONSTRAINTS_H

