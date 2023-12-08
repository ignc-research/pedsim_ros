#include "wall.h"
#include <pedsim_msgs/Obstacle.h>

#ifndef _scene_obstacle_h_
#define _scene_obstacle_h_

struct Obstacle{
    pedsim_msgs::Obstacle obstacle;
    std::vector<Wall*> walls;
};

#endif // _scene_obstacle_h_