/**
* kinematic_constraints.h
*/

#ifndef KINEMATIC_CONSTRAINTS_H
#define KINEMATIC_CONSTRAINTS_H

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

    bool IsStateWithinConstraints(CarState state) {
        if (state.get_vel() > max_vel || state.get_vel() < 0) {
            return false;
        }
        if (state.get_alpha() > max_alpha || state.get_alpha() < -max_alpha) {
            return false;
        }
        // TODO check lateral accel
        
        return true;
    }
};

#endif // KINEMATIC_CONSTRAINTS_H

