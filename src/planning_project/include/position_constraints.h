
#ifndef ALL_CONSTRAINTS_H
#define ALL_CONSTRAINTS_H

#include "car_state.h"
#include "kinematic_constraints.h"
#include "nav_msgs/msg/occupancy_grid.hpp"

class DynamicObstacleConstraint {
public:
    DynamicObstacleConstraint(const CarState &obstacle_start_state);
    bool isCollisionFree(const CarState &new_state, const CarState &old_state);
};

class PathConstraint {
    PathConstraint(const nav_msgs::msg::OccupancyGrid &map);
    bool isCollisionFree(const CarState &new_state, const CarState &old_state);
};

#endif // ALL_CONSTRAINTS_H
