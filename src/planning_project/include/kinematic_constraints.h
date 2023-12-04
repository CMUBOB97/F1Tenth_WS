/**
* kinematic_constraints.h
*/

#ifndef KINEMATIC_CONSTRAINTS_H
#define KINEMATIC_CONSTRAINTS_H

class KinematicConstraints {
public:
    float max_brake_accel;
    float max_gas_accel;
    float max_vel;
    float max_steer_angle;
    float max_lateral_accel;

    KinematicConstraints(float max_brake_accel, float max_gas_accel, float max_vel, float max_steer_angle, float max_lateral_accel) {
        this->max_brake_accel = max_brake_accel;
        this->max_gas_accel = max_gas_accel;
        this->max_vel = max_vel;
        this->max_steer_angle = max_steer_angle;
        this->max_lateral_accel = max_lateral_accel;
    }

    bool StateWithinConstraints(CarState state) {
        if (state.vel > max_vel || state.vel < 0) {
            return false;
        }
        if (state.alpha > max_steer_angle || state.alpha < -max_steer_angle) {
            return false;
        }
        // TODO check lateral accel
        return true;
    }
};

#endif // KINEMATIC_CONSTRAINTS_H

