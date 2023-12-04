/**
* CarState.h
*/

#ifndef CAR_STATE_H
#define CAR_STATE_H

// constants used for kinematic updates
const float dt = 0.1;
const float pi = 3.141592653589793;
const float wheel_base = 0.3302;

struct CarState {
    float t;
    float x;
    float y;
    float yaw;
    float vel;
    float alpha; 
};

typedef struct CarState CarState;

// update 
CarState CalcNextState(CarState state, float accel, float delta_alpha);

#endif // CAR_STATE_H
